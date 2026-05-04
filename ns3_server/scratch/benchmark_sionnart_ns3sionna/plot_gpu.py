import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def plot_gpu_usage(csv_file, output_image):
    df = pd.read_csv(csv_file)
    
    # Set style
    sns.set_theme(style="darkgrid")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot Peak Memory
    sns.lineplot(data=df, x="num_stas", y="peak_mem_mib", hue="backend", marker="o", ax=ax1)
    ax1.set_title("Peak GPU Memory Usage")
    ax1.set_xlabel("Number of Stations")
    ax1.set_ylabel("Memory (MiB)")
    ax1.set_xscale("log", base=2)
    
    # Plot Peak Utilization
    sns.lineplot(data=df, x="num_stas", y="peak_util_percent", hue="backend", marker="o", ax=ax2)
    ax2.set_title("Peak GPU Utilization")
    ax2.set_xlabel("Number of Stations")
    ax2.set_ylabel("Utilization (%)")
    ax2.set_xscale("log", base=2)
    ax2.set_ylim(0, 100)
    
    plt.tight_layout()
    plt.savefig(output_image)
    print(f"Plot saved to {output_image}")

if __name__ == "__main__":
    import sys
    csv_file = sys.argv[1] if len(sys.argv) > 1 else "gpu_usage_sweep.csv"
    output_image = sys.argv[2] if len(sys.argv) > 2 else "gpu_usage_plot.png"
    plot_gpu_usage(csv_file, output_image)
