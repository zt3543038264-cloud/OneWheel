"""Route protocol demonstration only; motion is an ideal polyline simulation."""
import math
import time
from app.services.routes import wire_points, route_crc


class MockPath:
    def __init__(self,transport):
        self.t=transport; self.points=(); self.saved=(); self.revision=1
        self.upload=None; self.token=0; self.arm=0; self.counter=0; self.arm_at=0
        self.active=False; self.state=0; self.progress=0.; self.speed=0.; self.dirty=0
        self.x=self.y=self.yaw=0.; self.stream=False

    def stop(self):
        self.upload=None; self.arm=0; self.active=False; self.speed=0
        self.state=2 if self.points else 0; self.revision+=1

    @property
    def length(self):
        return sum(math.dist(a,b) for a,b in zip(self.points,self.points[1:]))

    def status(self,ms):
        segment=0; distance=self.progress
        for i,(a,b) in enumerate(zip(self.points,self.points[1:])):
            d=math.dist(a,b); segment=i
            if distance<=d or i==len(self.points)-2:
                f=min(1,distance/d); self.x=a[0]+f*(b[0]-a[0]); self.y=a[1]+f*(b[1]-a[1])
                self.yaw=math.degrees(math.atan2(b[0]-a[0],b[1]-a[1])); break
            distance-=d
        self.t.append('nav:'+','.join(map(str,[ms,self.state,0,len(self.points),segment,self.x,self.y,self.yaw,self.progress,self.length,self.speed,self.dirty,int(bool(self.points)),int(self.active),self.revision])))

    def tick(self,ms):
        if self.active and self.state==3:
            self.progress=min(self.length,self.progress+self.speed*.02)
            if self.progress>=self.length:self.state=5; self.speed=0
        if self.stream and self.t.index%10==0:self.status(ms)

    def command(self,op,seq,args):
        if not op.startswith('path_'):return False
        prefix=f'rsp:{seq},'
        def ok(*fields):self.t.append(prefix+'ok,'+','.join(map(str,[op,*fields])))
        try:
            if op=='path_status':
                self.stream=True; self.status(int((time.monotonic()-self.t.origin)*1000)); ok(1,256,'1.50'); return True
            if op=='path_cancel':self.upload=None; self.arm=0; ok(); return True
            if self.t.mode!='STOP':raise ValueError('RUNNING_LOCKED')
            if op=='path_begin':
                count=int(args[0])
                if not 2<=count<=256:raise ValueError('BAD_FORMAT')
                self.upload=[]; self.expected=count; self.token=int(seq); self.upload_at=time.monotonic(); ok(self.token)
            elif op=='path_point':
                token,index,x,y=map(int,args)
                if self.upload is None or token!=self.token or index!=len(self.upload) or time.monotonic()-self.upload_at>5:raise ValueError('PATH_SESSION')
                if abs(x)>1000000 or abs(y)>1000000:raise ValueError('OUT_OF_RANGE')
                self.upload.append((x,y)); self.upload_at=time.monotonic(); ok(index)
            elif op=='path_commit':
                token,crc=map(int,args)
                if self.upload is None or token!=self.token or len(self.upload)!=self.expected or route_crc(self.upload)!=crc or time.monotonic()-self.upload_at>5:raise ValueError('PATH_SESSION')
                points=tuple((x/1000,y/1000) for x,y in self.upload); wire_points(points)
                self.points=points; self.upload=None; self.revision+=1; self.state=2; self.progress=0; self.dirty=1; ok(self.revision)
            elif op=='path_get':
                revision,index=map(int,args)
                if revision!=self.revision:raise ValueError('PATH_CHANGED')
                ok(index,*self.points[index])
            elif op=='path_arm':
                if int(args[0])!=self.revision or self.length<.3 or self.upload is not None:raise ValueError('INVALID_PATH')
                self.counter+=1; self.arm=self.counter; self.arm_at=time.monotonic(); self.arm_revision=self.revision; ok(self.arm)
            elif op=='path_start':
                token=int(args[0]); speed=float(args[1]); arm=self.arm; self.arm=0
                if not token or token!=arm or self.arm_revision!=self.revision or time.monotonic()-self.arm_at>3:raise ValueError('PATH_SESSION')
                if not .05<=speed<=1.5:raise ValueError('START_BLOCKED')
                self.active=True; self.state=3; self.speed=speed; self.progress=0; self.t.mode='Path'; self.t.manual_stopped=True; ok()
            elif op=='path_save':self.saved=self.points; self.dirty=0; ok('VERIFIED')
            elif op=='path_load':
                if not self.saved:raise ValueError('FLASH_FAILED')
                self.points=self.saved; self.revision+=1; self.state=2; self.dirty=0; ok('VERIFIED')
            else:raise ValueError('UNKNOWN_COMMAND')
        except (ValueError,IndexError,TypeError) as exc:
            self.upload=None; self.arm=0; self.t.append(prefix+'err,PATH_REJECTED,'+str(exc).replace(',',';'))
        return True
