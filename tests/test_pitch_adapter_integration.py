"""Independent coordinate-contract regression through the real RA/EKF shim.

The imported solver is booted normally with mock hardware. We then supply
already-solved quaternion/body-rate states to its real publication functions.
This tests publication, axis aliases and controller projection together; it
does not claim to test sensor filtering, real-time execution or vehicle safety.
"""
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def _function(source, name):
    match = re.search(r"^static\s+\w+\s+" + name + r"\([^\n]*\)\s*\{", source, re.M)
    assert match, name
    end = source.index("{", match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def test_real_ekf_publication_pitch_coordinate_contract(tmp_path):
    control = (ROOT / "code/Controller/control.c").read_text()
    integration = (ROOT / "tests/new_ekf/integration.c").read_text()
    assert "int main(void)" in integration
    # Reuse hardware stubs and the successful boot path, not their test cases.
    prefix = integration.split("static void tick(void)", 1)[0]
    prefix += _function(integration, "tick") + "\n" + _function(integration, "boot")
    declarations = "\n".join(
        line for line in control.splitlines()
        if re.match(r"static (?:float|uint8) .*s_pitch_", line)
    )
    source = prefix + "\n" + r'''
#include "HW_Drivers/IMU/imu.c"
#include "Estimation/Attitude/attitude.c"
''' + declarations + "\n"
    for name in ("ctrl_is_finite", "pitch_coordinates_valid", "pitch_euler_rate"):
        source += _function(control, name) + "\n"
    source += r'''
static void coordinate_case(float phi_deg,float theta_deg,
                            float phidot,float thetadot,float psidot)
{
    float phi=phi_deg*DEG_TO_RAD,theta=theta_deg*DEG_TO_RAD;
    float e[3],a[3]={0,0,1},g[3];
    float cr=cosf(phi/2),sr=sinf(phi/2),cp=cosf(theta/2),sp=sinf(theta/2);
    ticks+=2000;
    assert(!master_check_fault());
    g_attitude.q[0]=cr*cp;g_attitude.q[1]=sr*cp;
    g_attitude.q[2]=cr*sp;g_attitude.q[3]=-sr*sp;
    attitude_euler(g_attitude.q,e);
    imu_roll_deg=e[0];imu_pitch_deg=e[1];imu_yaw_deg=e[2];
    /* Forward native ZYX kinematics, independent of the controller inverse. */
    g[0]=phidot-psidot*sinf(theta);
    g[1]=thetadot*cosf(phi)+psidot*sinf(phi)*cosf(theta);
    g[2]=-thetadot*sinf(phi)+psidot*cosf(phi)*cosf(theta);
    g_attitude.input_ready=1;
    for(int i=0;i<3;i++) {
        g_attitude.gyro_lpf[i]=g[i]*DEG_TO_RAD;
        g_attitude.accel_lpf[i]=a[i];
    }
    imu_publish_navigation(g,a,.002f);
    publish_legacy_sample(master_imu_ticks());
    attitude_update_rate();attitude_update();
    assert(attitude_converged());
    assert(fabsf(att.pitch-phi_deg)<.0001f);
    assert(fabsf(att.roll-theta_deg)<.0001f);
    assert(fabsf(att.pitch_rate-g[0])<.0001f);
    assert(fabsf(att.roll_rate-g[1])<.0001f);
    assert(pitch_coordinates_valid());
    float got=pitch_euler_rate();
    if(!isfinite(got)||fabsf(got-phidot)>.001f)
        fprintf(stderr,"phi=%g theta=%g phidot=%g thetadot=%g psidot=%g got=%g\n",
                phi_deg,theta_deg,phidot,thetadot,psidot,got);
    assert(isfinite(got) && fabsf(got-phidot)<=.001f);
}
int main(void)
{
    const float phis[]={-30,-10,0,10,30},thetas[]={-20,-8,0,8,20};
    unsigned count=0;
    boot();tick();tick();
    for(unsigned i=0;i<5;i++) for(unsigned j=0;j<5;j++)
    for(int p=-20;p<=20;p+=20) for(int t=-10;t<=10;t+=10)
    for(int y=-120;y<=120;y+=120) {
        coordinate_case(phis[i],thetas[j],p,t,y);count++;
    }
    /* Original reported defect: steady tilted heading change, no Pitch drift. */
    coordinate_case(0,8,0,0,80);
    /* Absolute coordinate limits do not depend on mechanical zero settings. */
    att.pitch=0;att.roll=79.999f;assert(pitch_coordinates_valid());
    att.roll=80;assert(!pitch_coordinates_valid());
    att.roll=-80;assert(!pitch_coordinates_valid());
    att.roll=NAN;assert(!pitch_coordinates_valid());
    att.roll=0;att.pitch=INFINITY;assert(!pitch_coordinates_valid());
    printf("PASS %u real-shim kinematic cases, reported defect and domain guards\n",count);
    return 0;
}
'''
    c_file = tmp_path / "pitch_adapter_integration.c"
    executable = tmp_path / "pitch_adapter_integration"
    c_file.write_text(source)
    subprocess.run([
        "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I" + str(ROOT / "tests/new_ekf/mocks"),
        "-I" + str(ROOT / "code"),
        "-I" + str(ROOT / "libraries/zf_device"),
        str(c_file), "-lm", "-o", str(executable),
    ], check=True)
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS 675 real-shim kinematic cases" in result.stdout
