"""Verify Qcar configured defaults through actual param_load_defaults; no MCU-readback claim."""
import json
import re
from test_lean_limits import ROOT, _compile_run, _function


def test_mcu_snapshot_with_documented_forward_frame_changes(tmp_path):
    snapshot=json.loads((ROOT/'tools/unicycle_station/docs/qcar-defaults.json').read_text(encoding='utf-8'))
    values=snapshot['parameters']
    assert snapshot['transport']=='serial' and snapshot['count']==len(values)==75
    source=(ROOT/'code/Common/param.c').read_text(encoding='utf-8')
    table=re.search(r'const param_desc_t g_param_table\[\] =\s*\{.*?^\};',source,re.M|re.S).group()
    order=re.findall(r'\{\s*"(\w+)"',table)
    assert set(order)==set(values)
    expected=','.join(repr(float(values[n]['value']))+'f' for n in order)
    c='''#include "config.h"
#include "Common/param.h"
#include "Common/param.h"
#include <assert.h>
#include <math.h>
param_t g_param;
volatile uint32 g_param_revision;
'''+table+'\n'+_function(source,'param_load_defaults')+'''
int main(void){
    const float expected[]={'''+expected+'''};
    param_load_defaults();
    assert(g_param_revision==1);
    assert(sizeof(g_param_table)/sizeof(g_param_table[0])==75);
    for(unsigned i=0;i<75;i++){
        const param_desc_t *d=&g_param_table[i];
        float v=d->is_float?*(float*)d->ptr:(float)*(int*)d->ptr;
        assert(isfinite(v) && v==expected[i] && v>=d->vmin && v<=d->vmax);
    }
    assert(BAL_TEST_FLY_LIMIT==FLYWHEEL_OUT_LIMIT);
    assert(BAL_TEST_DRIVE_LIMIT==DRIVE_OUT_LIMIT);
    assert(DIRECTION_CAMERA_LIMIT==15000);
    return 0;
}
'''
    _compile_run(tmp_path,c)
    for name in ('app/profiles/tc264_unicycle/profile.json','examples/default.workspace.json'):
        d=json.loads((ROOT/'tools/unicycle_station'/name).read_text(encoding='utf-8'));p=d.get('profile',d)
        assert {v['name']:v['value'] for v in p['mock_parameters']}=={n:v['value'] for n,v in values.items()}
        assert all(p['parameters'][n]['default']==v['value'] for n,v in values.items())
