import os
import sys
from scapy.all import rdpcap
import matplotlib.pyplot as plt

def find_pcap_files(directory):
    return [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.pcap')]

def extract_throughput(pcap_path, bin_size=1):
    packets = rdpcap(pcap_path)
    if not packets:
        return [], []

    start_time = packets[0].time
    times = [pkt.time - start_time for pkt in packets]
    sizes = [len(pkt) for pkt in packets]

    max_time = int(times[-1]) + 1
    bins = [0] * max_time
    for t, s in zip(times, sizes):
        bins[int(t)] += s * 8 / 1e6  # bits to Mbps

    return list(range(max_time)), bins

def plot_throughput(x, y, label):
    plt.plot(x, y, label=label)

# === Main ===
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("❌ Please provide a folder path containing .pcap files.")
        print("✅ Usage: python script.py /path/to/pcap_folder")
        sys.exit(1)

    folder = sys.argv[1]
    if not os.path.isdir(folder):
        print(f"❌ Folder '{folder}' does not exist.")
        sys.exit(1)

    pcap_files = find_pcap_files(folder)
    if not pcap_files:
        print("⚠️ No .pcap files found in the specified folder.")
        sys.exit(0)

    plt.figure(figsize=(10, 5))
    for file in pcap_files:
        print(f"Processing {file}...")
        x, y = extract_throughput(file)
        if x:
            plot_throughput(x, y, label=os.path.basename(file))

    plt.xlabel("Time (s)")
    plt.ylabel("Throughput (Mbps)")
    plt.title("Throughput over Time from PCAPs")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # ✅ Save to the same folder as the .pcap files
    output_path = os.path.join(folder, "throughput_plot.png")
    plt.savefig(output_path)
    print(f"✅ Plot saved to: {output_path}")

    plt.show()
