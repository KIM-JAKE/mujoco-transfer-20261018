"""Intern bit-identical local class CAD AFTER the complete original attachment.

Only local proxy identities are reused. Every current grasp/pose, original USD
query write, actor filter and collision verdict remains the original operation.
"""
import functools
import hashlib
import struct
import inspect
from pathlib import Path
import numpy as np


def _snapshot(rows):
    if not isinstance(rows,(list,tuple))or not rows:raise ValueError('Unknown class rows')
    result=[]
    for row in rows:
        if not isinstance(row,tuple)or len(row)!=3:raise ValueError('Unknown class row')
        body,source,geometry=row
        if type(body)is not str or type(source)is not str or not isinstance(geometry,tuple)or len(geometry)!=5:raise ValueError('Unknown local class CAD')
        fields=[]
        for index,array in enumerate(geometry):
            if type(array)is not np.ndarray or array.dtype.kind!='f'or array.dtype.itemsize not in (4,8)or not np.isfinite(array).all():raise ValueError('Unsupported local CAD field')
            if index in (0,2)and array.shape!=(3,):raise ValueError('Invalid local OBB vector')
            if index==1 and array.shape!=(3,3):raise ValueError('Invalid local OBB axes')
            if index in (3,4)and(array.ndim!=2 or array.shape[1]!=3 or len(array)<(4 if index==3 else 1)):raise ValueError('Invalid local hull')
            fields.append((array.dtype.str,array.shape,array.strides,array.tobytes(order='C')))
        if np.any(geometry[2]<0)or np.max(abs(geometry[1].T@geometry[1]-np.eye(3)))>1e-5:raise ValueError('Invalid local OBB')
        result.append((body,source,tuple(fields)))
    return tuple(result)


def _digest(snapshot):
    h=hashlib.sha256(b'full-local-class-CAD-content-v1')
    def put(value):
        h.update(struct.pack('!Q',len(value)));h.update(value)
    for body,source,fields in snapshot:
        put(body.encode('utf8'));put(source.encode('utf8'))
        for dtype,shape,strides,data in fields:
            put(dtype.encode('ascii'));put(repr((shape,strides)).encode('ascii'));put(data)
    return h.digest()


MAX_RAW_INPUT_BYTES=1024*1024


def _raw_snapshot(vertices):
    if type(vertices)is not np.ndarray or vertices.nbytes>MAX_RAW_INPUT_BYTES or vertices.dtype.kind!='f'or vertices.dtype.itemsize not in (4,8)or vertices.ndim!=2 or vertices.shape[1]!=3 or len(vertices)<4 or not np.isfinite(vertices).all():
        raise ValueError('Unsupported original proxy input')
    return(vertices.dtype.str,vertices.shape,vertices.strides,vertices.tobytes(order='C'))


def _raw_digest(snapshot,source_sha):
    dtype,shape,strides,data=snapshot
    h=hashlib.sha256(source_sha.encode('ascii'))
    for value in(dtype.encode('ascii'),repr((shape,strides)).encode('ascii'),data):
        h.update(struct.pack('!Q',len(value)));h.update(value)
    return h.digest()


def _enable_raw_proxy(original_attach,stats):
    # This is ONLY the alias read by original Checker methods. The original
    # predictive_pair_clearance.proxy function/global module is not modified.
    namespace=original_attach.__globals__;original_proxy=namespace.get('proxy')
    if not callable(original_proxy):return dict(enabled=False,reason='Original proxy alias unavailable')
    try:
        source=Path(inspect.getsourcefile(original_proxy));source_sha=hashlib.sha256(source.read_bytes()+original_proxy.__qualname__.encode()+original_proxy.__code__.co_code).hexdigest()
    except Exception:return dict(enabled=False,reason='Original proxy implementation identity unavailable')
    cache={}

    @functools.wraps(original_proxy)
    def cached_proxy(*args,**kwargs):
        stats['raw_proxy_calls']+=1
        eligible=False;snapshot=None;key=None
        if len(args)==1 and not kwargs:
            try:
                snapshot=_raw_snapshot(args[0]);key=(source_sha,_raw_digest(snapshot,source_sha));eligible=True
                prior=cache.get(key)
                if prior is not None:
                    saved,raw_reference,output,output_snapshot=prior
                    try:unchanged=(_raw_snapshot(raw_reference)==saved and _snapshot([('proxy','proxy',output)])==output_snapshot)
                    except Exception:unchanged=False
                    if not unchanged:
                        cache.pop(key,None);stats['raw_proxy_cached_poisoned']+=1
                    elif snapshot!=saved:
                        stats['raw_proxy_digest_collision_or_mismatch']+=1;eligible=False
                    else:
                        stats['raw_proxy_hits']+=1
                        return output
            except Exception:
                stats['raw_proxy_unsupported']+=1;eligible=False
        else:stats['raw_proxy_unsupported']+=1
        # Exactly one original evaluation for misses, changed/unknown inputs,
        # poison and collisions. Its errors are NEVER caught or retried here.
        stats['raw_proxy_original_calls']+=1
        result=original_proxy(*args,**kwargs)
        if eligible:
            try:
                output_snapshot=_snapshot([('proxy','proxy',result)])
                # The original implementation may retain aliases on a hull
                # fallback. Do not cache if input bytes changed during it.
                if _raw_snapshot(args[0])!=snapshot:raise ValueError('Original proxy input changed')
                if len(cache)>=256:cache.clear()
                cache[key]=(snapshot,args[0],result,output_snapshot)
                stats['raw_proxy_new_content']+=1
            except Exception:stats['raw_proxy_unsupported_output']+=1
        return result

    namespace['proxy']=cached_proxy
    return dict(enabled=True,source_function_sha256=source_sha,scope='isaac_collision_queries original module alias only',predictive_original_function_unchanged=True,cache_entries_maximum=256,raw_input_bytes_maximum=MAX_RAW_INPUT_BYTES)


def enable(Checker):
    previous=getattr(Checker,'_payload_proxy_content_intern_info',None)
    if previous is not None:return previous
    original=Checker.attach_payload
    stats=dict(attachments=0,interned=0,new_content=0,invalid=0,cached_content_poisoned=0,digest_collision_or_mismatch=0,raw_proxy_calls=0,raw_proxy_hits=0,raw_proxy_original_calls=0,raw_proxy_new_content=0,raw_proxy_cached_poisoned=0,raw_proxy_digest_collision_or_mismatch=0,raw_proxy_unsupported=0,raw_proxy_unsupported_output=0)
    info=dict(enabled=True,original_attachment_and_query_writes_preserved=True,local_geometry_only=True,posed_collision_results_cached=False,exact_full_field_confirmation=True,cached_content_revalidated=True,stats=stats)

    @functools.wraps(original)
    def attach_payload(self,*args,**kwargs):
        # Original failures propagate with their original type/message. No
        # state is reused until all original source reads/query writes finish.
        result=original(self,*args,**kwargs)
        stats['attachments']+=1
        try:
            rows=self.payload_boxes
            path=self.payload_path
            grasp=np.asarray(self.payload_relative)
            if type(path)is not str or grasp.shape!=(4,4)or grasp.dtype.kind!='f'or not np.isfinite(grasp).all():raise ValueError('Unknown current attachment')
            snapshot=_snapshot(rows);key=(path,_digest(snapshot))
            cache=getattr(self,'_payload_proxy_content_intern_cache',None)
            if cache is None:cache={};self._payload_proxy_content_intern_cache=cache
            prior=cache.get(key)
            if prior is None:
                if len(cache)>=8:cache.clear()
                # Both immutable snapshots and strong original geometry refs
                # are retained. No geometry identity is trusted on its own.
                cache[key]=(snapshot,tuple(rows));stats['new_content']+=1
                return result
            saved,cached_rows=prior
            try:cached_snapshot=_snapshot(cached_rows)
            except Exception:cached_snapshot=None
            if cached_snapshot!=saved:
                cache.pop(key,None);stats['cached_content_poisoned']+=1
                return result
            if snapshot!=saved:
                stats['digest_collision_or_mismatch']+=1
                return result
            # Preserve the original outer list, body/source objects, current
            # path, relative grasp and payload_rows/query operations exactly.
            if type(rows)is not list:raise ValueError('Unsupported payload row container')
            replacements=[(body,source,cached[2])for(body,source,_),cached in zip(rows,cached_rows)]
            rows[:]=replacements;stats['interned']+=1
        except Exception:
            # Any malformed/unsupported metadata retains original fresh CAD.
            stats['invalid']+=1
        return result

    info['raw_proxy_cache']=_enable_raw_proxy(original,stats)
    Checker.attach_payload=attach_payload
    Checker._payload_proxy_content_intern_info=info
    return info
