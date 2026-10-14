from pathlib import Path
import tempfile
import unittest
from tf_route_audit.core import audit, input_records


class TfTests(unittest.TestCase):
    def test_route_both_directions_and_exact_ns(self):
        rows = [{'parent':'map','child':'base','static':True},
                {'parent':'base','child':'camera','stamp_ns':10**18},
                {'parent':'base','child':'camera','stamp_ns':10**18+1}]
        report = audit(rows, query=['camera','map'], at_ns=10**18+1)
        self.assertEqual(report['route']['frames'], ['camera','base','map'])
        self.assertTrue(report['route']['time_window_passed'])

    def test_static_ignores_stamp(self):
        report = audit([{'parent':'a','child':'b','static':True,'stamp_ns':0}], query=['a','b'],at_ns=900)
        self.assertEqual(report['error_count'],0)

    def test_outside_window(self):
        report = audit([{'parent':'a','child':'b','stamp_ns':10}],query=['a','b'],at_ns=11)
        self.assertFalse(report['route']['time_window_passed'])

    def test_cycle_multiple_parent_and_required(self):
        report = audit([{'parent':'a','child':'b'}, {'parent':'b','child':'a'}, {'parent':'c','child':'b'}],required=['missing'])
        self.assertEqual(set(report['finding_counts']), {'cycle','multiple_parents','missing_required_frame'})

    def test_disconnected_route_is_not_claimed(self):
        report = audit([{'parent':'a','child':'b'}, {'parent':'c','child':'d'}],query=['a','d'])
        self.assertIsNone(report['route'])
        self.assertIn('missing_route',report['finding_counts'])

    def test_bad_quaternion_and_time_regression(self):
        report = audit([{'parent':'a','child':'b','stamp_ns':2,'rotation':[0,0,0,0]},
                        {'parent':'a','child':'b','stamp_ns':1}])
        self.assertEqual(report['finding_counts'],{'non_unit_quaternion':1,'time_regression':1})

    def test_ros_transformstamped(self):
        row={'header':{'frame_id':'a','stamp':{'sec':2,'nanosec':3}},'child_frame_id':'b',
             'transform':{'translation':{'x':0,'y':0,'z':0},'rotation':{'x':0,'y':0,'z':0,'w':1}}}
        self.assertEqual(audit([row])['edges'][0]['min_ns'],2_000_000_003)
        row['header']['stamp']['nanosec']=1_000_000_000
        with self.assertRaises(ValueError): audit([row])

    def test_native_urdf_preserves_isolated_links(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'robot.urdf'
            path.write_text('<robot name="r"><link name="a"/><link name="b"/><link name="c"/>'
                            '<joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint></robot>')
            rows,links=input_records(path)
            report=audit(rows,query=['a','b'],at_ns=1,declared_frames=links)
            self.assertEqual(len(report['components']),2)
            self.assertFalse(report['route']['time_window_passed'])

    def test_large_chain_avoids_recursion_depth(self):
        rows=[{'parent':str(x),'child':str(x+1),'static':True} for x in range(1500)]
        self.assertEqual(len(audit(rows,query=['0','1500'])['route']['frames']),1501)
