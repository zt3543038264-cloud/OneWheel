"""Compare the complete station schema with executable firmware descriptors.

These are host contracts, not a target build or a hardware readback.
"""
import json
import re
import subprocess
import sys

import pytest
from test_lean_limits import ROOT, _compile_run, _function

STATION = ROOT / 'tools/unicycle_station'


def test_generated_profiles_and_defaults_are_reproducible():
    result = subprocess.run([sys.executable, str(STATION / 'scripts/generate_profiles.py'), '--check'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize('relative', [
    'app/profiles/tc264_unicycle/profile.json', 'examples/default.workspace.json',
])
def test_complete_schema_order_type_ranges_defaults_and_permissions(tmp_path, relative):
    data = json.loads((STATION / relative).read_text(encoding='utf-8'))
    profile = data.get('profile', data)
    params = profile['mock_parameters']
    assert len(params) == 78
    assert len({p['name'] for p in params}) == len(params)
    assert [p['name'] for p in params] == list(profile['parameters'])
    param = (ROOT / 'code/Common/param.c').read_text(encoding='utf-8')
    protocol = (ROOT / 'code/Protocols/vofa.c').read_text(encoding='utf-8')
    table = re.search(r'const param_desc_t g_param_table\[\] =\s*\{.*?^\};', param, re.M | re.S).group()
    pid_names = re.search(r'static const char \*const s_cfg_pid_names\[\] = \{.*?^\};', protocol, re.M | re.S).group()
    def function(name):
        return re.search(r'^static [^\n]+ ' + name + r'\([^\n]*\)\s*\{.*?^\}',
                         protocol, re.M | re.S).group()
    # cfg_group's pointer attaches to the function name in the production source.
    group = re.search(r'^static const char \*cfg_group\([^\n]*\)\s*\{.*?^\}',
                      protocol, re.M | re.S).group()
    def f(value):
        return repr(float(value)) + 'f'
    rows = []
    for p in params:
        assert profile['parameters'][p['name']]['default'] == p['value']
        rows.append('{"%s", "%s", %d, %s, %s, %s, %d}' % (
            p['name'], p['group'], p['type'] == 'float', f(p['min']),
            f(p['max']), f(p['value']), p['flags']))
    source = '''#include "Common/board_config.h"
#include "Common/param.h"
#include <assert.h>
#include <math.h>
#include <string.h>
param_t g_param;
volatile uint32 g_param_revision;
''' + table + '\n' + _function(param, 'param_load_defaults') + '\n'
    source += pid_names + '\n' + function('cfg_pid_index') + '\n'
    source += function('cfg_live_control') + '\n' + group + '\n' + function('cfg_dangerous')
    source += '''
struct expected_t { const char *name, *group; int floating; float lo, hi, value; unsigned flags; };
int main(void) {
    const struct expected_t expected[] = {''' + ',\n'.join(rows) + '''};
    const unsigned count = sizeof(g_param_table)/sizeof(g_param_table[0]);
    assert(count == sizeof(expected)/sizeof(expected[0]));
    param_load_defaults();
    for (unsigned i=0; i<count; i++) {
        const param_desc_t *d = &g_param_table[i];
        const struct expected_t *e = &expected[i];
        float v = d->is_float ? *(float *)d->ptr : (float)*(int *)d->ptr;
        unsigned flags = 4u | (cfg_pid_index(d->name) >= 0 ? 1u : 0u) |
            (cfg_live_control(d->name) ? 17u : 0u) | (cfg_dangerous(d->name) ? 2u : 0u);
        assert(strcmp(d->name,e->name)==0 && strcmp(cfg_group(d->name),e->group)==0);
        assert(d->is_float==e->floating && d->vmin==e->lo && d->vmax==e->hi);
        assert(isfinite(v) && v==e->value && v>=d->vmin && v<=d->vmax);
        assert(flags==e->flags);
    }
    return 0;
}
'''
    _compile_run(tmp_path, source)


def test_mock_advertises_schema_without_inventing_flags():
    from app.core.models import DeviceProfile
    from app.protocols.firewater import FireWaterParser
    from app.transports.mock import MockTransport
    profile = DeviceProfile.load(STATION / 'app/profiles/tc264_unicycle/profile.json')
    mock = MockTransport(profile)
    expected = profile.data['mock_parameters']
    assert list(mock.params) == [p['name'] for p in expected]
    assert list(mock.params.values()) == expected
    mock.open()
    try:
        mock.write(b'cfg:schema,2\r\n')
        parser = FireWaterParser()
        parser.feed(bytes(mock.queue), received=0)
        frames = list(parser.parsed_frames())
        schema = [f for f in frames if f.tag == 'par']
        assert len(schema) == len(expected) == 78
        for frame, param in zip(schema, expected):
            actual = profile.parameter(frame)
            assert (actual.name, actual.type, actual.min, actual.max, actual.value) == (
                param['name'], param['type'], param['min'], param['max'], param['value'])
            assert int(frame.values[-1]) == param['flags']
        assert any(f.tag == 'rsp' and f.values == ['2', 'ok', 'schema', '78'] for f in frames)
    finally:
        mock.close()
    # A stale/custom profile must not gain permissions merely from a known name.
    profile.data['mock_parameters'][0]['flags'] = 4
    assert MockTransport(profile).params['run_speed_straight']['flags'] == 4


def test_synthetic_demo_session_uses_current_schema():
    profile = json.loads((STATION / 'app/profiles/tc264_unicycle/profile.json').read_text(encoding='utf-8'))
    demo = STATION / 'examples/demo_session'
    session = json.loads((demo / 'session.json').read_text(encoding='utf-8'))
    assert session['profile'] == profile
    assert '不是硬件测量' in session['description']
    frames = [json.loads(line) for line in (demo / 'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    schema = [frame for frame in frames if frame.get('tag') == 'par']
    assert [frame['values'][1] for frame in schema] == [p['name'] for p in profile['mock_parameters']]
    assert len(schema) == 78
    raw = (demo / 'raw.txt').read_text(encoding='utf-8')
    assert 'rsp:2,ok,schema,78\n' in raw
    assert len([line for line in raw.splitlines() if line.startswith('par:2,')]) == 78
