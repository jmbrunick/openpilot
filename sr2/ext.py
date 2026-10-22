import json,os,sys,capnp,zstandard,pickle
capnp.remove_import_hook(); CD='/tmp/cer19'; log=capnp.load(CD+"/log.capnp",imports=[CD,CD+"/include"])
seg,src=sys.argv[1],sys.argv[2]
raw=zstandard.ZstdDecompressor().stream_reader(open(src,'rb')).read()
enc=[];v=[];cal=[];md=[];gps=[]
for e in log.Event.read_multiple_bytes(raw,traversal_limit_in_words=2**40):
  try: w=e.which()
  except Exception: continue
  t=e.logMonoTime*1e-9
  if w=='roadEncodeIdx': x=e.roadEncodeIdx; enc.append((t,x.segmentId,x.frameId))
  elif w=='carState': v.append((t,e.carState.vEgo))
  elif w=='liveCalibration': x=e.liveCalibration; cal.append((t,str(x.calStatus),list(x.rpyCalib)))
  elif w=='modelV2':
    x=e.modelV2
    if not md or t-md[-1][0]>0.09:
      try: md.append((t,x.frameId,list(x.roadEdges[1].x),list(x.roadEdges[1].y),float(x.roadEdgeStds[1]),float(x.velocity.x[0]) if len(x.velocity.x) else None))
      except Exception: pass
  elif w in('gpsLocationExternal','gpsLocation'):
    x=getattr(e,w); gps.append((t,x.speed,bool(x.hasFix) if hasattr(x,'hasFix') else True))
pickle.dump(dict(enc=enc,v=v,cal=cal,md=md,gps=gps),open(f'/workspace/sr2/ctx/{seg}.pkl','wb'))
print(seg,len(enc),len(v),len(cal),len(md),len(gps))
