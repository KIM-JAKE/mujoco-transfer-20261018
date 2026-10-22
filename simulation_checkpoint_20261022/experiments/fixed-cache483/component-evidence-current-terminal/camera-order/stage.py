from pathlib import Path
import ast,difflib,json
base=Path('/tmp/fixed479-full/overrides/loose_ring_pick.py');s=base.read_text();out=Path('/tmp/fixed482-large-camera-order')
a="""    def observe(mode,part=None,G=None,allow_missing=False):
        capture_gap="""
b="""    def observe(mode,part=None,G=None,allow_missing=False):
        # The distant pre-approach observation keeps its old view order. Only
        # a nearby fixed LARGE pickup uses the two formerly successful views
        # first; every original view and fresh-image consistency gate remains.
        prefer_fixed_large=bool(fixed_supply and mode=='pregrasp_ring' and part=='ring1' and G is not None and c['config'].get('fixed_large_pregrasp_preferred_views',False) and np.linalg.norm(np.asarray(G)[:3,3])<=.10)
        capture_gap="""
assert s.count(a)==1;s=s.replace(a,b)
a="if mode=='pregrasp_ring' and part=='ring2':";b="if mode=='pregrasp_ring' and (part=='ring2' or prefer_fixed_large):";assert s.count(a)==1;s=s.replace(a,b)
a="""                    offsets=[np.array([-.28,-.20,.32]),old_base]+offsets
                if c['config'].get('preclose_occlusion_max_age_s')"""
b="""                    offsets=[np.array([-.28,-.20,.32]),old_base]+offsets
                elif prefer_fixed_large:
                    old_base=short*.38+np.array([0.,0.,.40])
                    offsets=[np.array([-.28,-.20,.32]),old_base]+offsets
                if c['config'].get('preclose_occlusion_max_age_s')"""
assert s.count(a)==1;s=s.replace(a,b);ast.parse(s);(out/'loose_ring_pick.py').write_text(s);(out/'loose_ring_pick.py.diff').write_text(''.join(difflib.unified_diff(base.read_text().splitlines(True),s.splitlines(True),fromfile='a/loose_ring_pick.py',tofile='b/loose_ring_pick.py')));(out/'config-delta.json').write_text(json.dumps({'fixed_large_pregrasp_preferred_views':True},indent=2)+'\n')
