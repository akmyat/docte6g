import subprocess
import shutil
from pathlib import Path

def run_simulations():
    # Configuration
    array_sizes = [2, 4, 8]
    load_types = ["low", "medium", "high"]
    sim_modes = ["sionna", "ns3"]
    # Multiple seeds help separate channel effects from one random UE placement.
    seeds = [41, 42, 43]
    sim_time = 30.0
    num_ues = 20
    gnb_tx_power_dbm = 30
    gnb_height = 10
    scheduler_type = "ns3::NrMacSchedulerTdmaPF"
    error_model_type = "ns3::NrEesmIrT2"
    ue_dist_min = 150
    ue_dist_max = 300
    # The gNB panel uses a +Y boresight in main.cc (bearingAngle = pi/2).
    # Keep UEs close to boresight for the single-panel showcase. Wider sectors
    # need multi-sector support to behave like a real 360-degree site.
    ue_azimuth_min_deg = 80
    ue_azimuth_max_deg = 100
    
    # Paths
    # This script is expected to be in ns3_server/scratch/sionnart-nr-energy/
    # We should run ns3 from the ns3_server root.
    script_dir = Path(__file__).parent.absolute()
    ns3_root = script_dir.parent.parent
    ns3_exe = ns3_root / "ns3"
    
    base_output_dir = Path("/home/aung/code/new_docte6g/results")
    
    failures = []
    for mode in sim_modes:
        for seed in seeds:
            for size in array_sizes:
                for load in load_types:
                    seed_suffix = f"_seed_{seed}" if len(seeds) > 1 else ""
                    test_name = f"{mode}_gnb_{size}x{size}_load_{load}{seed_suffix}"
                    output_dir = base_output_dir / test_name
                    if output_dir.exists():
                        shutil.rmtree(output_dir)
                    output_dir.mkdir(parents=True, exist_ok=True)

                    print(f"\n>>> Starting Test: {test_name}", flush=True)

                    cmd = [
                        str(ns3_exe), "run",
                        (
                            f"sionnart-nr-urban-macro --simMode={mode} "
                            f"--gnbAntennaRows={size} --gnbAntennaCols={size} "
                            f"--loadType={load} --seed={seed} "
                            f"--numUes={num_ues} "
                            f"--gnbTxPowerDbm={gnb_tx_power_dbm} "
                            f"--gnbHeight={gnb_height} "
                            f"--schedulerType={scheduler_type} "
                            f"--errorModelType={error_model_type} "
                            f"--ueDist_min={ue_dist_min} --ueDist_max={ue_dist_max} "
                            f"--ueAzimuthMinDeg={ue_azimuth_min_deg} "
                            f"--ueAzimuthMaxDeg={ue_azimuth_max_deg} "
                            f"--outputDir={str(output_dir)} --simTime={sim_time}"
                        )
                    ]

                    result = subprocess.run(
                        cmd,
                        cwd=str(ns3_root),
                        capture_output=True,
                        text=True,
                    )
                    (output_dir / "run_stdout.log").write_text(result.stdout)
                    (output_dir / "run_stderr.log").write_text(result.stderr)
                    if result.returncode == 0:
                        print(f"<<< Finished Test: {test_name}. Results in {output_dir}")
                        continue

                    failures.append(test_name)
                    print(f"!!! Error running test {test_name}: exit code {result.returncode}")
                    if result.stdout:
                        print(f"STDOUT:\n{result.stdout}")
                    if result.stderr:
                        print(f"STDERR:\n{result.stderr}")

    if failures:
        raise SystemExit(f"{len(failures)} simulation(s) failed: {', '.join(failures)}")

if __name__ == "__main__":
    run_simulations()
