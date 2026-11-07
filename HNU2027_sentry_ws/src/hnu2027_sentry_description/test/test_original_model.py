"""Regression checks against the original MJCF export manifest."""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as E

BASE=Path(__file__).resolve().parents[1]/'resource/models/hnu2027_sentry'
M=E.parse(BASE/'model.sdf').getroot().find('model')
REF=json.loads((BASE/'source_manifest.json').read_text())
def values(node): return [float(x) for x in node.text.split()]

class OriginalModelTest(unittest.TestCase):
    def close(self,a,b):
        self.assertEqual(len(a),len(b))
        for x,y in zip(a,b): self.assertAlmostEqual(x,y,places=6)

    def test_body_positions_and_total_mass(self):
        total=0
        for b in REF['bodies']:
            n=M.find(f"link[@name='{b['name']}']") if b['mass'] else M.find(f"frame[@name='{b['name']}']")
            self.assertIsNotNone(n)
            self.close(values(n.find('pose'))[:3],b['pos'])
            self.assertEqual(n.find('pose').get('relative_to'),b['parent'])
            if b['mass']:
                total+=float(n.find('inertial/mass').text)
                self.assertEqual(float(n.find('inertial/mass').text),b['mass'])
                self.assertTrue(all(float(n.find('inertial/inertia/'+x).text)>0 for x in ('ixx','iyy','izz')))
        self.assertAlmostEqual(total,24.815)

    def test_all_original_visuals_and_collision_selection(self):
        visuals={n.get('name'):n for n in M.findall('link/visual')}
        collisions={n.get('name'):n for n in M.findall('link/collision')}
        self.assertEqual(set(visuals),{g['name'] for g in REF['geoms']})
        self.assertEqual(set(collisions),{g['name']+'_collision' for g in REF['geoms'] if g['collision']})
        for g in REF['geoms']:
            n=visuals[g['name']]
            self.close(values(n.find('pose'))[:3],g['pos'])
            self.assertEqual(n.find('pose').get('relative_to'),g['body'])
            self.close(values(n.find('material/diffuse')),g['rgba'])
            geo=n.find('geometry')
            if g['kind']==6: self.close(values(geo.find('box/size')),[2*x for x in g['size']])
            elif g['kind'] in (3,5):
                shape=geo.find('capsule' if g['kind']==3 else 'cylinder')
                self.assertAlmostEqual(float(shape.find('radius').text),g['size'][0])
                self.assertAlmostEqual(float(shape.find('length').text),2*g['size'][1])

    def test_joint_axes_limits_and_hierarchy(self):
        for j in REF['joints']:
            n=M.find(f"joint[@name='{j['name']}']")
            self.assertEqual(n.get('type'),'revolute')
            self.assertEqual(n.find('parent').text,j['parent']); self.assertEqual(n.find('child').text,j['child'])
            self.close(values(n.find('axis/xyz')),j['axis'])
            self.close([float(n.find('axis/limit/'+k).text) for k in ('lower','upper')],j['limits'])
        fold=M.find("joint[@name='fold_roll']")
        self.close(values(fold.find('axis/xyz')),[1,0,0])
        self.assertAlmostEqual(float(fold.find('axis/limit/upper').text),math.pi/2,places=6)
        self.assertEqual(M.find("joint[@name='mid360_fixed']/parent").text,'upper_platform')

    def test_wheels_and_rollers(self):
        for corner in ('fl','fr','rl','rr'):
            wheel=M.find(f"link[@name='wheel_{corner}']")
            self.assertIsNotNone(wheel)
            self.assertEqual(len([v for v in wheel.findall('visual') if '_roller_' in v.get('name')]),12)

    def test_control_topics_are_original_rotational_joints(self):
        plugins=M.findall("plugin[@name='ignition::gazebo::systems::JointPositionController']")
        self.assertEqual({p.find('joint_name').text for p in plugins},{'big_yaw','fold_roll','small_yaw','pitch'})
        for p in plugins:
            self.assertEqual(p.find('topic').text,'/model/hnu2027_sentry/joint/'+p.find('joint_name').text+'/cmd_pos')


    def test_body_and_geometry_orientations(self):
        def quat_matrix(q):
            w,x,y,z=q
            return [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
                    2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
                    2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]
        def pose_matrix(node):
            r,p,y=values(node)[3:]
            cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
            return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
                    sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,
                    -sp,cp*sr,cp*cr]
        for b in REF['bodies']:
            node=M.find(f"link[@name='{b['name']}']") if b['mass'] else M.find(f"frame[@name='{b['name']}']")
            self.close(pose_matrix(node.find('pose')),quat_matrix(b['quat']))
        visuals={n.get('name'):n for n in M.findall('link/visual')}
        for g in REF['geoms']:
            self.close(pose_matrix(visuals[g['name']].find('pose')),quat_matrix(g['quat']))

if __name__=='__main__': unittest.main()
