"""Verify current profile defaults through production param_init; no MCU-readback claim.

docs/qcar-defaults.json is a historical 75-field serial capture, not a current
schema after the Roll PI, encoder, Beacon and Yaw migrations.
"""
import json
import re
from test_lean_limits import ROOT, _compile_run, _function
from test_imu_forward import param_host_source


def test_current_profiles_match_production_defaults(tmp_path):
    path=ROOT/'tools/unicycle_station/app/profiles/tc264_unicycle/profile.json'
    profile=json.loads(path.read_text(encoding='utf-8'))
    values={p['name']:p for p in profile['mock_parameters']}
    source=(ROOT/'code/Common/param.c').read_text(encoding='utf-8')
    table=re.search(r'const param_desc_t g_param_table\[\] =\s*\{.*?^\};',source,re.M|re.S).group()
    order=re.findall(r'\{\s*"(\w+)"',table)
    assert set(order)==set(values)
    expected=','.join(repr(float(values[n]['value']))+'f' for n in order)
    c=param_host_source()+'''
int main(void){
    const float expected[]={'''+expected+'''};
    param_init();
    assert(sizeof(g_param_table)/sizeof(g_param_table[0])==sizeof(expected)/sizeof(expected[0]));
    for(unsigned i=0;i<sizeof(expected)/sizeof(expected[0]);i++){
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
