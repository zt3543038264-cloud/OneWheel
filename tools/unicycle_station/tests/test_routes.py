import json
import math
import time
from pathlib import Path
import pytest
from app.services.routes import resample, wire_points, route_crc, document, read_route
from app.protocols.firewater import FireWaterParser
from app.services.engine import StationEngine
from app.ui.window import builtin_profile


def test_route_geometry_document_and_wire():
    points=resample([(0,0),(0,5),(2,7)])
    wire=wire_points(points)
    assert points[0]==(0,0) and points[-1]==(2,7)
    assert max(math.dist(a,b) for a,b in zip(points,points[1:]))<.202
    assert read_route(json.loads(json.dumps(document(points,1.5))))['speed_mps']==1.5
    parser=FireWaterParser()
    assert len(parser.encode_command('path_point',65535,65535,255,-1000000,-1000000).strip())<=63
    assert route_crc([(0,0),(0,200),(0,400)])==1240199005
    for invalid in [[(0,0),(0,float('nan'))],[(1,0),(2,0)],[(0,0),(0,0)],[(0,0),(0,1000)]]:
        with pytest.raises(ValueError):resample(invalid)
    with pytest.raises(ValueError):document(points,1.51)
    with pytest.raises(ValueError):read_route({'kind':'unknown'})
    assert parser.parse('nav:1,3,0,10,0,0,0,0,0,3,1.5,0,1,1,5',1).tag=='nav'
    assert parser.parse('nav:1,99,0,10,0,0,0,0,0,3,1.5,0,1,1,5',1).error


def wait_for(predicate,timeout=8):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('timed out')


def test_route_mock_engine_roundtrip_start_stop_and_failure():
    e=StationEngine(builtin_profile());e.auto_record=False
    try:
        e.start('mock')
        wait_for(lambda:e.routes.supported is True)
        e.emergency_stop();wait_for(lambda:e.stopped() and not e.stop_requested.is_set())
        points=resample([(0,0),(0,3)])
        e.submit('path','upload',points)
        wait_for(lambda:e.routes.car_revision is not None)
        revision=e.routes.car_revision
        wait_for(lambda:e.routes.nav and int(e.routes.nav[14])==revision)
        assert e.routes.car_route==points
        e.submit('path','save');wait_for(lambda:e.transport.path.saved==points)
        wait_for(lambda:not e.routes.busy)
        e.submit('path','load')
        wait_for(lambda:e.transport.path.revision!=revision and not e.routes.busy)
        assert not e.routes.car_route
        wait_for(lambda:int(e.routes.nav[14])==e.transport.path.revision)
        revision=int(e.routes.nav[14])
        e.submit('path','download');wait_for(lambda:bool(e.routes.car_route) and not e.routes.busy)
        assert e.routes.car_route==points
        e.submit('path','start',1.5,revision)
        wait_for(lambda:e.transport.path.active)
        wait_for(lambda:not e.routes.busy)
        assert e.stop_message==''
        wait_for(lambda:e.routes.nav and e.routes.nav[1]==5)
        assert e.routes.nav[13]==1  # completed while keeping balance
        e.emergency_stop();wait_for(lambda:e.stopped() and not e.stop_requested.is_set())
        wait_for(lambda:not e.requests.pending and not e.requests.queue)
        wait_for(lambda:e.routes.nav and not e.routes.nav[13])
        e.transport.command_fault='reject'
        e.submit('path','upload',resample([(0,0),(0,4)]))
        wait_for(lambda:'操作未确认' in e.routes.message)
        assert e.transport.path.points==points
        assert not e.routes.busy
    finally:e.close()


def test_route_page_offscreen():
    from PySide6.QtWidgets import QApplication
    app=QApplication.instance() or QApplication([])
    from app.ui.route_page import RoutePage
    e=StationEngine(builtin_profile());page=RoutePage(e);page.show();app.processEvents()
    page.length.setValue(10);page.straight()
    assert page.points[-1]==(0,10)
    assert page.speed.maximum()==1.5
    page.update_data();assert not page.start.isEnabled()
    page.clear();page.add_point(point=(0,3));assert page.generate()
    assert page.points[-1]==(0,3)

    page.close();page.deleteLater();app.processEvents()
