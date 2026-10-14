"""Offline graph evidence, deliberately distinct from a live tf2 buffer."""
from collections import defaultdict, deque
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .common import Findings, integer, load, number, text


def input_records(path):
    path = Path(path)
    if path.suffix.lower() in {'.urdf', '.xml'}:
        if path.stat().st_size > 16 * 1024 * 1024:
            raise ValueError('URDF exceeds 16 MiB.')
        root = ET.parse(path).getroot()
        if root.tag != 'robot':
            raise ValueError('Expected a URDF robot element; xacro is not evaluated.')
        links = [text(link.get('name'), 'link name') for link in root.findall('link')]
        if len(set(links)) != len(links):
            raise ValueError('Duplicate URDF link names.')
        rows = []
        for joint in root.findall('joint'):
            parent, child = joint.find('parent'), joint.find('child')
            if parent is None or child is None:
                raise ValueError('URDF joint requires parent and child.')
            a, b = parent.get('link'), child.get('link')
            if a not in links or b not in links:
                raise ValueError('URDF joint references an undeclared link.')
            rows.append({'parent': a, 'child': b, 'static': joint.get('type') == 'fixed',
                         'stamp_ns': None, 'source': 'urdf'})
        return rows, links
    data = load(path)
    if isinstance(data, dict):
        if 'messages' in data:
            rows = []
            for message in data['messages']:
                if not isinstance(message, dict) or not isinstance(message.get('transforms'), list):
                    raise ValueError('Each message requires a transforms list.')
                for transform in message['transforms']:
                    row = dict(transform)
                    row['static'] = message.get('topic') == '/tf_static'
                    if 'authority' in message:
                        row['authority'] = message['authority']
                    rows.append(row)
            return rows, []
        data = data.get('transforms')
    if not isinstance(data, list):
        raise ValueError('Use a transforms list, {transforms:[...]}, or {messages:[...]}.')
    return data, []


def normalize(row):
    if not isinstance(row, dict):
        raise ValueError('Transform must be an object.')
    static = row.get('static', False)
    if not isinstance(static, bool):
        raise ValueError('static must be a boolean.')
    if 'header' in row:
        header = row['header']
        stamp = header['stamp']
        sec = integer(stamp['sec'], 'stamp.sec')
        nano = integer(stamp['nanosec'], 'stamp.nanosec')
        if nano >= 1_000_000_000:
            raise ValueError('nanosec must be less than 1e9.')
        value = sec * 1_000_000_000 + nano
        transform = row['transform']
        return dict(parent=text(header['frame_id'], 'parent'),
                    child=text(row['child_frame_id'], 'child'), static=static, stamp_ns=value,
                    rotation=[transform['rotation'][key] for key in 'xyzw'],
                    translation=[transform['translation'][key] for key in 'xyz'],
                    authority=row.get('authority'))
    value = row.get('stamp_ns')
    if value is not None:
        integer(value, 'stamp_ns')
    return dict(row, parent=text(row.get('parent'), 'parent'), child=text(row.get('child'), 'child'),
                static=static, stamp_ns=value)


def audit(rows, required=(), query=None, at_ns=None, declared_frames=()):
    if at_ns is not None:
        integer(at_ns, 'at_ns')
    findings = Findings()
    edges = defaultdict(list)
    frames = set(declared_frames)
    if len(frames) > 10_000:
        raise ValueError('Frame limit of 10,000 exceeded.')
    parents = defaultdict(set)
    for index, raw in enumerate(rows):
        if index >= 1_000_000:
            raise ValueError('Transform limit of 1,000,000 exceeded.')
        row = normalize(raw)
        a, b = row['parent'], row['child']
        frames.update((a, b))
        if len(frames) > 10_000:
            raise ValueError('Frame limit of 10,000 exceeded.')
        parents[b].add(a)
        if a == b:
            findings.add('self_transform', 'error', 'A frame cannot be its own parent.', frame=a)
        for name, size in [('rotation', 4), ('translation', 3)]:
            if name in row:
                values = row[name]
                if not isinstance(values, list) or len(values) != size:
                    raise ValueError(f'{name} requires {size} values.')
                values = [number(value, name) for value in values]
                if name == 'rotation' and abs(math.sqrt(sum(x*x for x in values)) - 1) > 1e-3:
                    findings.add('non_unit_quaternion', 'error', 'Quaternion norm differs from one.', edge=[a,b])
        prior = edges[(a, b)]
        if prior and row.get('stamp_ns') is not None and prior[-1].get('stamp_ns') is not None:
            if not row['static'] and row['stamp_ns'] < prior[-1]['stamp_ns']:
                findings.add('time_regression', 'warning', 'Dynamic TF stamp decreases in observed order.', edge=[a,b])
        prior.append(row)
    if not frames:
        findings.add('empty_input', 'error', 'No frames were observed.')
    for child, candidates in sorted(parents.items()):
        if len(candidates) > 1:
            findings.add('multiple_parents', 'error', 'Aggregated evidence gives a child multiple parents; inspect time-separated reparenting.',
                         child=child, parents=sorted(candidates))
    for edge, samples in edges.items():
        if {s['static'] for s in samples} == {True, False}:
            findings.add('static_dynamic_conflict', 'error', 'An edge was recorded as both static and dynamic.', edge=list(edge))
    adjacency = defaultdict(set)
    directed = defaultdict(set)
    indegree = dict.fromkeys(frames, 0)
    for a, b in edges:
        adjacency[a].add(b); adjacency[b].add(a)
        directed[a].add(b); indegree[b] += 1
    pending = deque(sorted(frame for frame, degree in indegree.items() if degree == 0))
    visited = set()
    while pending:
        a = pending.popleft(); visited.add(a)
        for b in directed[a]:
            indegree[b] -= 1
            if indegree[b] == 0:
                pending.append(b)
    if visited != frames:
        findings.add('cycle', 'error', 'Directed cycle exists; listed frames include cycle-dependent descendants.',
                     affected_frames=sorted(frames - visited))
    components, unseen = [], set(frames)
    while unseen:
        start = min(unseen); component = {start}; todo = [start]
        while todo:
            for nxt in adjacency[todo.pop()] - component:
                component.add(nxt); todo.append(nxt)
        unseen -= component; components.append(sorted(component))
    if len(components) > 1:
        findings.add('disconnected_components', 'warning', 'More than one disconnected frame component was observed.', count=len(components))
    for frame in required:
        if frame not in frames:
            findings.add('missing_required_frame', 'error', 'Required frame is absent.', frame=frame)
    route = None
    if query:
        if len(query) != 2:
            raise ValueError('Query needs source and target frames.')
        source, target = query
        todo, back = deque([source]), {source: None}
        while todo:
            current = todo.popleft()
            if current == target:
                break
            for nxt in sorted(adjacency[current]):
                if nxt not in back:
                    back[nxt] = current; todo.append(nxt)
        if source not in frames or target not in back or target not in frames:
            findings.add('missing_route', 'error', 'No observed topology route connects the requested frames.', query=list(query))
        else:
            chain, current = [], target
            while current is not None:
                chain.append(current); current = back[current]
            chain.reverse()
            route = {'frames': chain, 'at_ns': at_ns, 'time_window_passed': True if at_ns is not None else None}
            for a, b in zip(chain, chain[1:]):
                samples = edges.get((a,b), edges.get((b,a)))
                if all(sample['static'] for sample in samples):
                    continue
                times = [sample['stamp_ns'] for sample in samples if sample.get('stamp_ns') is not None]
                if at_ns is not None and (not times or not min(times) <= at_ns <= max(times)):
                    route['time_window_passed'] = False
                    findings.add('outside_observed_window', 'error', 'Query time is outside this dynamic edge evidence.', edge=[a,b])
    edge_summary = []
    for (a,b), samples in sorted(edges.items()):
        times = [s['stamp_ns'] for s in samples if s.get('stamp_ns') is not None]
        edge_summary.append(dict(parent=a, child=b, samples=len(samples), static=all(s['static'] for s in samples),
                                 min_ns=min(times) if times else None, max_ns=max(times) if times else None,
                                 authorities=sorted({s['authority'] for s in samples if isinstance(s.get('authority'),str)})))
    return dict(tool='tf-route-audit', schema_version=1, frames=sorted(frames), components=components,
                edges=edge_summary, route=route, semantics='Observed topology/time ranges only; no tf2 cache or interpolation simulation.',
                **findings.report())
