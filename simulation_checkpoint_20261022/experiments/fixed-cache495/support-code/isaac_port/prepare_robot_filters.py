"""Export MuJoCo welded-body, parent-body and explicit robot exclusions."""
import json,hashlib
from pathlib import Path
import mujoco
from prepare import HERE,SOURCES
m=mujoco.MjModel.from_xml_path(str(SOURCES['cap_button']));ids=[i for i in range(m.nbody)if m.body(i).name.startswith(('left_','right_','official_FR3_Duo'))]
explicit={tuple(sorted((int(v)>>16,int(v)&65535)))for v in m.exclude_signature};pairs=[]
parent_filter=not(int(m.opt.disableflags)&int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT))
for n,a in enumerate(ids):
 for b in ids[n+1:]:
  wa,wb=int(m.body_weldid[a]),int(m.body_weldid[b]);pa=int(m.body_weldid[m.body_parentid[wa]]);pb=int(m.body_weldid[m.body_parentid[wb]])
  why='same welded component'if wa==wb else'explicit source exclude'if(a,b)in explicit else'non-world parent welded component'if parent_filter and((wa!=0 and wa==pb)or(wb!=0 and wb==pa))else None
  if why:pairs.append(dict(body0=m.body(a).name,body1=m.body(b).name,reason=why))
report=dict(source_sha256=hashlib.sha256(SOURCES['cap_button'].read_bytes()).hexdigest(),parent_filter=parent_filter,pairs=pairs)
(HERE/'generated/contact_task/robot-filters.json').write_text(json.dumps(report,indent=2)+'\n');print('Robot collision exclusion pairs:',len(pairs))
