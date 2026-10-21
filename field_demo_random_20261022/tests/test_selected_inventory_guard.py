from datetime import datetime, timedelta, timezone
import copy
import unittest
import numpy as np
from field_demo.selected_inventory_guard import verify_single_inventory, verify_empty_hand

NOW=datetime(2026,10,21,18,0,tzinfo=timezone.utc)


def models():
    theta=np.linspace(0,2*np.pi,360,endpoint=False);result={}
    for name,outer,inner,half in [('ring1',.0375,.0315,.00775),('ring2',.031,.02525,.007)]:
        points=[]
        for z in [-half,half]:
            for radius in np.linspace(inner,outer,12):points.append(np.c_[radius*np.cos(theta),radius*np.sin(theta),np.full(len(theta),z)])
        for radius in [outer,inner]:
            for z in np.linspace(-half,half,16):points.append(np.c_[radius*np.cos(theta),radius*np.sin(theta),np.full(len(theta),z)])
        result[name]=np.concatenate(points)
    return result


def image(index, flipped=False, extra=False, empty=False):
    width=192;f=1000.;cx=(width-1)/2;yy,xx=np.indices((width,width));depth=np.full((width,width),2.)
    plane=1.-.007;px=(xx-cx)*plane/f;py=(yy-cx)*plane/f;rr=np.hypot(px,py)
    if not empty:depth[(rr>=.02525)&(rr<=.031)]=plane
    if extra:
        other=np.hypot(px-.055,py);depth[(other>=.02525)&(other<=.031)]=plane
    part=np.eye(4);part[:3,3]=[0,0,1.]
    if flipped:part[:3,:3]=np.diag([1.,-1.,-1.])
    g=np.eye(4);g[:3,3]=[0,0,.02];tcp=part@np.linalg.inv(g)
    return dict(packet=dict(depth_m=depth,K=np.array([[f,0,cx],[0,f,cx],[0,0,1.]]),T_cell_camera=np.eye(4)),
                metadata=dict(capture_id='view'+str(index),captured_at_utc=(NOW-timedelta(seconds=.1*(1-index))).isoformat(),
                              source_packet_sha256='a'*64,calibration_sha256='b'*64,source='synthetic_RGBD_test',replay=True,
                              rectified_registered_depth=True,holding_epoch='actual_test_grasp1'),T_cell_part=part,T_cell_tcp=tcp)


def hand(grasped=True):
    return dict(completed=True,is_grasped=grasped,width_m=.062 if grasped else .08,max_width_m=.08,robot_time_s=1.)


class SelectedInventoryTests(unittest.TestCase):
    def test_one_ring_two_actual_views_one_camera_and_stable_grasp(self):
        views=[image(0),image(1,flipped=True)]
        result=verify_single_inventory('ring2',views,models(),hand(),now_utc=NOW)
        self.assertTrue(result['verified'],result)
        self.assertGreaterEqual(min(v['cap_coverage_deg']for v in result['views']),240)
        self.assertEqual(result['maximum_grasp_translation_change_m'],0.)

    def test_single_or_repeated_view_cannot_hide_stacked_item(self):
        for views in [[image(0)],[image(0),image(1)]]:
            result=verify_single_inventory('ring2',views,models(),hand(),now_utc=NOW)
            self.assertFalse(result['verified'])

    def test_broad_roi_catches_extra_ring_outside_nearest_selected_crop(self):
        result=verify_single_inventory('ring2',[image(0,extra=True),image(1,flipped=True,extra=True)],models(),hand(),now_utc=NOW)
        self.assertFalse(result['verified']);self.assertIn('not explained by ONE',result['reason'])

    def test_wrong_class_slip_missing_depth_and_sdk_failure_reject(self):
        views=[image(0),image(1,flipped=True)]
        self.assertFalse(verify_single_inventory('ring1',views,models(),hand(),now_utc=NOW)['verified'])
        slipping=copy.deepcopy(views);slipping[1]['T_cell_tcp'][0,3]+=.004
        self.assertFalse(verify_single_inventory('ring2',slipping,models(),hand(),now_utc=NOW)['verified'])
        missing=copy.deepcopy(views);missing[0]['packet']['depth_m'][30:160,30:160]=np.nan
        self.assertFalse(verify_single_inventory('ring2',missing,models(),hand(),now_utc=NOW)['verified'])
        self.assertFalse(verify_single_inventory('ring2',views,models(),hand(False),now_utc=NOW)['verified'])
        self.assertFalse(verify_single_inventory('ring2',views,models(),hand(),now_utc=NOW+timedelta(seconds=3))['verified'])

    def test_self_geometry_exclusion_requires_current_actual_sdk_model(self):
        views=[image(0),image(1,flipped=True)];views[0]['known_robot_points_cell_m']=np.zeros((10,3))
        result=verify_single_inventory('ring2',views,models(),hand(),now_utc=NOW)
        self.assertFalse(result['verified']);self.assertIn('Self-geometry',result['reason'])

    def test_actual_open_hand_and_broad_empty_roi_after_release(self):
        views=[image(0,empty=True),image(1,flipped=True,empty=True)]
        result=verify_empty_hand(views,hand(False),now_utc=NOW)
        self.assertTrue(result['verified'],result)
        views[1]=image(1,flipped=True)
        self.assertFalse(verify_empty_hand(views,hand(False),now_utc=NOW)['verified'])
        closed=hand(False);closed['width_m']=.04
        self.assertFalse(verify_empty_hand([image(0,empty=True),image(1,flipped=True,empty=True)],closed,now_utc=NOW)['verified'])


if __name__=='__main__':unittest.main()
