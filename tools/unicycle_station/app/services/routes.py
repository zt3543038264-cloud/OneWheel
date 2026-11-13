"""Metre-based route documents and acknowledged, transactional path transfers."""
from collections import deque
import math
import struct
import time
import zlib


def wire_points(points, minimum_length=.30):
    if not 2 <= len(points) <= 256:
        raise ValueError('路线需要 2–256 个点')
    out = []
    total = 0.
    for point in points:
        if len(point) != 2 or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or abs(v)>1000 for v in point):
            raise ValueError('坐标必须是有限数，范围 ±1000 m')
        p = tuple(round(v*1000) for v in point)
        if not out and p != (0, 0):
            raise ValueError('第一个点必须为原点 (0, 0)')
        if out:
            distance = math.dist(p, out[-1])/1000
            if not .005 <= distance <= .999:
                raise ValueError('路线相邻点距离须为 0.005–0.999 m；请先生成路线')
            total += distance
        out.append(p)
    if not minimum_length <= total <= 1000:
        raise ValueError('可执行路线总长须为 0.30–1000 m')
    return tuple(out)


def resample(waypoints, spacing=.20):
    if not 2 <= len(waypoints) <= 256 or not .05 <= spacing <= .5:
        raise ValueError('请添加至少两个航点，间距为 0.05–0.50 m')
    for p in waypoints:
        if len(p)!=2 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>1000 for v in p):
            raise ValueError('航点坐标无效')
    if tuple(waypoints[0]) != (0,0):
        raise ValueError('起点必须为 (0, 0)')
    result=[(0.,0.)]
    for a,b in zip(waypoints,waypoints[1:]):
        distance=math.dist(a,b)
        if distance<.005: raise ValueError('相邻航点不能重合')
        count=max(1,math.ceil(distance/spacing-.01))
        if len(result)+count>256: raise ValueError('超过 256 点；增大采样间距或缩短路线')
        result.extend((a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count) for i in range(1,count+1))
    return tuple((x/1000,y/1000) for x,y in wire_points(result))


def route_crc(points):
    return zlib.crc32(b''.join(struct.pack('<ii',x,y) for x,y in points))


def document(points, speed=.20):
    wire_points(points)
    if isinstance(speed,bool) or not isinstance(speed,(float,int)) or not math.isfinite(speed) or not .05<=speed<=1.50:
        raise ValueError('速度范围 0.05–1.50 m/s')
    return dict(kind='unicycle_route',schema_version=1,units='m',axes='x_right_y_forward',speed_mps=speed,points=points)


def read_route(data):
    if not isinstance(data,dict) or data.get('kind')!='unicycle_route' or data.get('schema_version')!=1 or data.get('units')!='m' or data.get('axes')!='x_right_y_forward':
        raise ValueError('需要米制 unicycle_route v1 路线文件')
    try: return document(data['points'],data['speed_mps'])
    except (KeyError,TypeError) as exc: raise ValueError('路线文件字段无效') from exc


class RouteService:
    """All commands run on engine I/O thread. GUI reads immutable snapshots."""
    def __init__(self,engine):
        self.engine=engine
        self.reset()

    def reset(self):
        self.supported=None
        self.busy=False
        self.nav=()
        self.nav_at=0.
        self.probe_at=0.
        self.message='连接新版固件后可上传与执行路线'
        self.car_route=()
        self.car_revision=None
        self.route_event=0
        self.trace=deque(maxlen=6000)
        self.last_uptime=None

    def accept(self,frame):
        if frame.tag!='nav' or frame.error: return
        with self.engine.lock:
            v=tuple(frame.values)
            if self.nav and v[14]!=self.nav[14] and v[14]!=self.car_revision and not v[13]:
                self.car_route=(); self.car_revision=None
            if self.nav and (v[0]<self.nav[0] or (v[1] in (1,3) and (self.nav[1]!=v[1] or v[8]<self.nav[8]))):
                self.trace.clear()
            if v[1] in (1,3,4,5) and v[0]!=self.last_uptime:
                self.trace.append((v[5],v[6]))
            self.last_uptime=v[0]
            self.nav=v; self.nav_at=frame.received

    def available(self):
        e=self.engine
        return bool(e.connected and e.protocol_ready and not e.replay and not e.stop_requested.is_set() and self.supported is True and time.monotonic()-self.nav_at<.8)

    def tick(self):
        e=self.engine
        if e.replay or not e.connected or not e.protocol_ready or self.supported is not None or self.busy: return
        if e.requests.pending or e.requests.queue or time.monotonic()-self.probe_at<2: return
        self.probe_at=time.monotonic()
        def reply(ok,fields):
            self.supported=bool(ok and fields[:4]==['path_status','1','256','1.50'])
            self.message='路线接口已连接' if self.supported else '此固件不支持路线接口，请更新车端'
        e.requests.enqueue('path_status',callback=reply)

    def _done(self,ok,fields,message='操作已确认'):
        self.busy=False
        self.message=message if ok else '操作未确认：'+','.join(map(str,fields))+'；请读回确认'
        self.engine.event('PATH_OK' if ok else 'PATH_FAILED',self.message)

    def action(self,op,*args):
        e=self.engine
        if op=='cancel':
            if self.busy: e.requests.cancel('CANCELLED')
            if e.protocol_ready and not e.replay: e.requests.enqueue('path_cancel',callback=lambda ok,f:self._done(ok,f,'传输已取消；请下载确认车内路线'))
            return
        if not self.available() or not e.stopped() or self.nav[1] in (1,3,4) or self.nav[13]:
            raise ValueError('请连接新版车端，停止车辆并等待新鲜状态')
        if self.busy or e.requests.pending or e.requests.queue: raise ValueError('等待当前请求完成')
        if op=='upload':
            points=wire_points(args[0]); route=tuple((x/1000,y/1000) for x,y in points)
            self.busy=True; self.message='开始上传，车内旧路线保留到提交成功'
            def begin(ok,fields):
                if not ok: self._done(ok,fields); return
                if len(fields)!=2 or not fields[1].isdigit() or not 1<=int(fields[1])<=65535:
                    self._done(False,["BAD_RESPONSE"]); return
                token=int(fields[1])
                def point(index):
                    self.message=f'上传路线 {index}/{len(points)}'
                    if index==len(points):
                        def committed(ok,fields):
                            ok=ok and len(fields)==2 and fields[1].isdigit() and 0<=int(fields[1])<=0xffffffff
                            if ok:
                                self.car_route=route; self.car_revision=int(fields[1]); self.route_event+=1
                            self._done(ok,fields,'路线已上传到 RAM；可执行或保存 Flash')
                        e.requests.enqueue('path_commit',token,route_crc(points),callback=committed); return
                    def accepted(ok,fields):
                        if ok and fields==['path_point',str(index)]: point(index+1)
                        else: self._done(False,fields)
                    e.requests.enqueue('path_point',token,index,*points[index],callback=accepted)
                point(0)
            e.requests.enqueue('path_begin',len(points),callback=begin)
        elif op=='download':
            revision=int(self.nav[14]); count=int(self.nav[3]); points=[]
            if count<2: raise ValueError('车内暂无有效路线')
            self.busy=True
            def point(index):
                self.message=f'下载路线 {index}/{count}'
                if index==count:
                    try: wire_points(points, minimum_length=.10)
                    except ValueError as exc: self._done(False,[str(exc)]); return
                    self.car_route=tuple(points); self.car_revision=revision; self.route_event+=1
                    self._done(True,[],'车内路线已下载'); return
                def accepted(ok,fields):
                    if not ok or len(fields)!=4 or fields[1]!=str(index): self._done(False,fields); return
                    try: p=(float(fields[2]),float(fields[3]))
                    except ValueError: self._done(False,["BAD_RESPONSE"]); return
                    if not all(math.isfinite(v) and abs(v)<=1000 for v in p):
                        self._done(False,["BAD_RESPONSE"]); return
                    points.append(p); point(index+1)
                e.requests.enqueue('path_get',revision,index,callback=accepted)
            point(0)
        elif op=='start':
            speed=float(args[0]); revision=int(args[1])
            if not math.isfinite(speed) or not .05<=speed<=1.5: raise ValueError('速度超出范围')
            if not self.car_route or revision!=self.car_revision or revision!=int(self.nav[14]):
                raise ValueError('车内路线已变化，请先下载并核对')
            self.busy=True
            def armed(ok,fields):
                if not ok: self._done(ok,fields); return
                if len(fields)!=2 or not fields[1].isdigit() or not 1<=int(fields[1])<=0xffffffff:
                    self._done(False,["BAD_RESPONSE"]); return
                if not self.available() or not e.stopped(): self._done(False,['STALE_STATUS']); return
                def started(ok,fields):
                    if ok: e.stop_message=''
                    self._done(ok,fields,'MCU 已接受路线启动')
                e.requests.enqueue('path_start',int(fields[1]),f'{speed:.2f}',callback=started)
            e.requests.enqueue('path_arm',revision,callback=armed)
        elif op in ('save','load'):
            self.busy=True
            if op=='load': self.car_revision=None; self.car_route=()
            e.requests.enqueue('path_'+op,callback=lambda ok,f:self._done(ok,f,'路线 Flash 操作已验证'))
        else: raise ValueError('未知路线操作')
