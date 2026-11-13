"""Static ADS configuration contracts, independent of a local IDE workspace.

Passing these checks does not replace TASKING compilation/link/map inspection.
"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


def configuration():
    document = ET.parse(ROOT / '.cproject')
    configs = document.findall('./storageModule/cconfiguration')
    assert len(configs) == 1, 'Only TASKING Debug is supported in this review package'
    config = configs[0].find('./storageModule/configuration')
    assert config is not None and config.get('name') == 'Debug'
    return document, config


def test_only_current_project_tasking_debug_is_supported():
    document, config = configuration()
    assert ET.parse(ROOT / '.project').findtext('name') == 'Qcar'
    assert 'tasking' in config.find('.//toolChain').get('superClass')
    assert config.find('.//builder').get('buildPath') == '${workspace_loc:/${ProjName}/Debug}'
    assert config.find('.//option[@superClass="com.tasking.ctc.cpu"]').get('value') == 'tc26xb'
    assert config.find('.//option[@superClass="com.tasking.ctc.package"]').get('value') == 'bga292'
    assert not document.findall('.//scannerConfigBuildInfo')
    refresh = document.findall('./storageModule[@moduleId="refreshScope"]/configuration')
    assert [r.get('configurationName') for r in refresh] == ['Debug']
    assert refresh[0].find('resource').get('workspacePath') == '/Qcar'
    assert 'ASC_SummerTraining_Unicycle' not in (ROOT / '.cproject').read_text()


def test_all_include_and_library_roots_exist_in_this_project():
    _, config = configuration()
    options = config.findall('.//option[@valueType="includePath"]')
    paths = [v.get('value').strip('"') for o in options for v in o.findall('listOptionValue')]
    assert len(paths) == len(set(paths)) == 86
    for path in paths:
        match = re.fullmatch(r'\$\{workspace_loc:/\$\{ProjName\}/(.*)\}', path)
        assert match, path
        assert (ROOT / match[1]).is_dir(), path
    props = dict(line.split('=', 1) for line in
                 (ROOT / '.settings/com.infineon.aurix.buildsystem.prefs').read_text().splitlines())
    assert props['LIBRARIES-ROOT-PATH'] == 'libraries'
    assert props['NEVER-EXCLUDE-FROM-BUILD'] == 'libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/CStart'
    for path in props['AURIX-LIBRARY-PATH'].split(r'\#'):
        assert (ROOT / path).is_dir(), path
    assert 'Libraries/' not in '\n'.join(props.values())


def test_link_archive_fpu_runtime_and_startup_owner_are_retained():
    document, config = configuration()
    linker = config.find('.//tool[@name="TASKING Linker"]')
    values = {v.get('value').strip('"') for v in linker.findall('.//listOptionValue')}
    assert {'-lcs_fpu', '-lfp_fpu', '-lrt', '--lsl-core=vtc'} <= values
    archive = 'libraries/zf_device/zf_device_config.a'
    assert '${workspace_loc:/${ProjName}/' + archive + '}' in values
    assert (ROOT / archive).is_file()
    assert (ROOT / 'Lcf_Tasking_Tricore_Tc.lsl').is_file()
    booster = document.find('.//storageModule[@moduleId="com.infineon.aurix.buildsystem.build.booster.settings"]')
    assert [p.text for p in booster.findall('never-exclude-from-build/path')] == [
        'libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/CStart']
    assert [p.text for p in booster.findall('libraries-roots/path')] == ['libraries']


def test_source_exclusions_and_new_ekf_include_ownership():
    _, config = configuration()
    entries = config.findall('sourceEntries/entry')
    assert len(entries) == 1 and entries[0].get('name') == ''
    excludes = set(entries[0].get('excluding').split('|'))
    assert {'tools', 'tests', 'validation', 'docs', 'build', '.venv',
            'Seekfree_CYT2BL3_Double_Foc_Project', 'code/Estimation/Attitude/r10h'} <= excludes
    def active(path):
        return not any(path == x or path.startswith(x + '/') for x in excludes)
    sources = [p for p in ROOT.rglob('*.c') if active(p.relative_to(ROOT).as_posix())]
    relative = {p.relative_to(ROOT).as_posix() for p in sources}
    assert {'code/HW_Drivers/IMU/imu.c', 'code/Estimation/Attitude/attitude.c',
            'user/cpu0_main.c', 'user/cpu1_main.c'} <= relative
    new = ROOT / 'code/Estimation/Attitude/new_ekf'
    assert not list(new.glob('*.c')), 'Imported units must not compile a second time'
    units = {'attitude.inc', 'attitude_algorithm.inc', 'imu.inc'}
    assert {p.name for p in new.glob('*.inc')} == units
    for unit in units:
        owners = []
        for source in sources:
            includes = re.findall(r'^\s*#\s*include\s*"([^"]+)"', source.read_text(errors='replace'), re.M)
            if any(Path(i).name == unit for i in includes):
                owners.append(source.relative_to(ROOT).as_posix())
        assert owners == ['code/HW_Drivers/IMU/imu.c'], (unit, owners)
