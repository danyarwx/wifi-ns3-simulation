// wifi_simulation.cc
//
// - Builds 1 AP + 3 STA nodes
// - Varies STA distance from AP across scenarios
// - Generates TCP traffic from each STA -> AP
// - Logs throughput, avg delay, and packet loss to CSV
// 
// Author: Danila Zhukov

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiDistanceStudy");

static void WriteCsvHeaderIfNeeded(const std::string &csvPath) {
  std::ifstream check(csvPath);
  bool exists = check.good();
  check.close();
  if (!exists) {
    std::ofstream out(csvPath, std::ios_base::out);
    out << "distance_m,throughput_mbps,avg_delay_ms,packet_loss_percent\n";
    out.close();
  }
}

int main (int argc, char *argv[]) {
  std::string csvPath = "results.csv";
  bool verbose = false;

  CommandLine cmd(__FILE__);
  cmd.AddValue("csv", "Output CSV filepath", csvPath);
  cmd.AddValue("verbose", "Enable verbose WiFi logging", verbose);
  cmd.Parse(argc, argv);

  if (verbose) {
    LogComponentEnable("WifiDistanceStudy", LOG_LEVEL_INFO);
  }

  // Distances (meters) of the STA cluster from the AP
  std::vector<double> distances = {5.0, 10.0, 20.0, 35.0, 50.0};

  // Simulation timing
  double appStart = 1.0;   // seconds
  double appStop  = 10.0;  // seconds
  double simStop  = 12.0;  // seconds

  WriteCsvHeaderIfNeeded(csvPath);

  for (double d : distances) {
    // --- Nodes ---
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(3);

    // --- PHY & Channel ---
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    // Default uses LogDistance + ConstantSpeed delay; good enough for a simple study
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
    phy.SetChannel(channel.Create());

    // --- MAC & standard ---
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a); // Simple and fast

    // Control the rate adaptation (Minstrel is default; keep defaults for simplicity)
    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-distance-ssid");

    // STA devices
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevices = wifi.Install(phy, mac, wifiStaNodes);

    // AP device
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice = wifi.Install(phy, mac, wifiApNode);

    // --- Mobility ---
    // AP at origin; STAs at (d, +/-3) and (d, 0) to avoid exact co-location
    MobilityHelper mobilityAp;
    Ptr<ListPositionAllocator> apPos = CreateObject<ListPositionAllocator>();
    apPos->Add(Vector(0.0, 0.0, 0.0));
    mobilityAp.SetPositionAllocator(apPos);
    mobilityAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAp.Install(wifiApNode);

    MobilityHelper mobilitySta;
    Ptr<ListPositionAllocator> staPos = CreateObject<ListPositionAllocator>();
    staPos->Add(Vector(d, 0.0, 0.0));
    staPos->Add(Vector(d, 3.0, 0.0));
    staPos->Add(Vector(d, -3.0, 0.0));
    mobilitySta.SetPositionAllocator(staPos);
    mobilitySta.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilitySta.Install(wifiStaNodes);

    // --- Internet stack & IPs ---
    InternetStackHelper internet;
    internet.Install(wifiApNode);
    internet.Install(wifiStaNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apIf = ipv4.Assign(apDevice);
    Ipv4InterfaceContainer staIf = ipv4.Assign(staDevices);

    // --- Applications ---
    // TCP sink on AP (listens on port 5000)
    uint16_t port = 5000;
    Address sinkAddr(InetSocketAddress(apIf.GetAddress(0), port));
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(wifiApNode.Get(0));
    sinkApp.Start(Seconds(appStart));
    sinkApp.Stop(Seconds(appStop));

    // Each STA runs BulkSend to the AP sink
    ApplicationContainer senders;
    for (uint32_t i = 0; i < wifiStaNodes.GetN(); ++i) {
      BulkSendHelper bulk("ns3::TcpSocketFactory", sinkAddr);
      bulk.SetAttribute("MaxBytes", UintegerValue(0)); // unlimited
      bulk.SetAttribute("SendSize", UintegerValue(1448)); // typical TCP payload
      ApplicationContainer a = bulk.Install(wifiStaNodes.Get(i));
      a.Start(Seconds(appStart + 0.1 * i)); // small staggers
      a.Stop(Seconds(appStop));
      senders.Add(a);
    }

    // --- Flow monitor for delay & loss ---
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    // --- Run ---
    Simulator::Stop(Seconds(simStop));
    Simulator::Run();

    // --- Metrics ---
    // Throughput: bytesRx at sink / (appStop - appStart)
    uint64_t totalRx = 0;
    // PacketSink only on AP[0]
    Ptr<Application> app = sinkApp.Get(0);
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(app);
    if (sink) {
      totalRx = sink->GetTotalRx();
    }

    double duration = appStop - appStart;
    double throughputMbps = (totalRx * 8.0) / (duration * 1e6); // Mbit/s

    // Use FlowMonitor for avg delay and packet loss %
    monitor->CheckForLostPackets();
    double sumDelaySec = 0.0;
    uint64_t rxPackets = 0;
    uint64_t txPackets = 0;
    uint64_t lostPackets = 0;

    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    for (const auto &kv : stats) {
      const FlowMonitor::FlowStats &st = kv.second;
      sumDelaySec += st.delaySum.GetSeconds();
      rxPackets += st.rxPackets;
      txPackets += st.txPackets;
      lostPackets += st.lostPackets;
    }

    double avgDelayMs = (rxPackets > 0) ? (sumDelaySec / rxPackets * 1000.0) : 0.0;
    double lossPct = (txPackets > 0) ? (100.0 * (double)lostPackets / (double)txPackets) : 0.0;

    // --- 10) Write CSV row ---
    std::ofstream out(csvPath, std::ios_base::app);
    out << std::fixed << std::setprecision(2)
        << d << "," << throughputMbps << ","
        << avgDelayMs << "," << lossPct << "\n";
    out.close();

    if (verbose) {
      NS_LOG_UNCOND("Distance " << d << " m"
                      << " | Thr " << throughputMbps << " Mbps"
                      << " | AvgDelay " << avgDelayMs << " ms"
                      << " | Loss " << lossPct << " %");
    }

    Simulator::Destroy(); // clean state for next distance
  }

  return 0;
}

