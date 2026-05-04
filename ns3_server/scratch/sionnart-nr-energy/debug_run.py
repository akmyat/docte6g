import subprocess
import shutil
from pathlib import Path

def run_simulations():
    # TEST ONLY 2 CASES
    array_sizes = [2]
    load_types = ["low"]
    sim_modes = ["sionna", "ns3"]
    seed = 42
    sim_time = 10.0
    
    script_dir = Path(__file__).parent.absolute()
    ns3_root = script_dir.parent.parent
    ns3_exe = ns3_root / "ns3"
    
    base_output_dir = script_dir / "results" / "debug_test"
    
    for mode in sim_modes:
        for size in array_sizes:
            for load in load_types:
                test_name = f"{mode}_gnb_{size}x{size}_load_{load}"
                output_dir = base_output_dir / test_name
                if output_dir.exists():
                    shutil.rmtree(output_dir)
                output_dir.mkdir(parents=True, exist_ok=True)
                
                print(f"\n>>> Starting Test: {test_name}")
                
                cmd = [
                    str(ns3_exe), "run",
                    f"sionnart-nr-energy --simMode={mode} --gnbAntennaRows={size} --gnbAntennaCols={size} --loadType={load} --seed={seed} --outputDir={str(output_dir)} --simTime={sim_time}"
                ]
                
                print(f"Executing: {' '.join(cmd)}")
                try:
                    result = subprocess.run(cmd, cwd=str(ns3_root), capture_output=True, text=True)
                    (output_dir / "run_stdout.log").write_text(result.stdout)
                    (output_dir / "run_stderr.log").write_text(result.stderr)
                    if result.returncode != 0:
                        print(f"!!! Error running test {test_name}: Exit code {result.returncode}")
                        print(f"STDOUT: {result.stdout}")
                        print(f"STDERR: {result.stderr}")
                    else:
                        print(f"<<< Finished Test: {test_name}. Results in {output_dir}")
                except Exception as e:
                    print(f"!!! Exception running test {test_name}: {e}")

if __name__ == "__main__":
    run_simulations()
