/*****************************************************************************/
// This program started life as tcp-variants-comparison.cc provided in the NS3
// Suite. tcp-variants-comparison.cc was written BY:
//            Justin P. Rohrer, Truc Anh N. Nguyen <annguyen@ittc.ku.edu>,
//            Siddharth Gangadhar <siddharth@ittc.ku.edu>
//
// All the modified done to this code is performed by Alan Lin. The implentation
// of this code is aimed to used existing tracing methodology and to add a few
// more trace sources to give access to the actual sampled RTT, delta between the
// sampled rtt and estimated RTT.
//
// NOTE: The "RTT" trace in the existing suite is mislabelled. The value returned
//       is actually the output of the rtt estimate. Modifications to
//       tcp-socket-base.c/h and rtt-estimator.c/h were made to make available the
//       trace for sampled RTT and the delta.
//
// TOPOLOGY:
//
//
// Can create N source and sink pair and Flows between each pair.
// Each pair has their own flow thro a shair pair of gateways that are used
// by other nodes in the topology.
//
// There are two sets of access deleys, one is assigned to odd flows
// the other is assigned to even flows
//
/*****************************************************************************/

#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include "ns3/yans-wifi-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/error-model.h"
#include "ns3/tcp-header.h"
#include "ns3/udp-header.h"
#include "ns3/enum.h"
#include "ns3/event-id.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "../src/internet/model/tcp-socket-base-1.h"
#include "ns3/ssid.h"
#include "ns3/mobility-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/aodv-module.h"




using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DumbbellDifferingFlows");

/*****************************************************************************/
/*Initialation and pointers to trace streams                                 */
/*****************************************************************************/
bool firstCwnd = true;
bool firstSshThr = true;
bool firstRtt = true;
bool firstRto = true;
bool firstRttVar = true;
bool firstRealRtt = true;
bool firstDelta = true;

bool firstRtt_2 = true;

Ptr<OutputStreamWrapper> cWndStream;
Ptr<OutputStreamWrapper> ssThreshStream;
Ptr<OutputStreamWrapper> rttStream;
Ptr<OutputStreamWrapper> rtoStream;

Ptr<OutputStreamWrapper> rttvarStream;
Ptr<OutputStreamWrapper> rrttStream;
Ptr<OutputStreamWrapper> deltaStream;

Ptr<OutputStreamWrapper> rttStream_2;

uint32_t cWndValue;
uint32_t ssThreshValue;

/*****************************************************************************/
/*CALLBACK FUNCTIONS FOR TRACES                                               */
/*****************************************************************************/
static void
CwndTracer (uint32_t oldval, uint32_t newval)
{
  if (firstCwnd)
    {
      *cWndStream->GetStream () << "0.0 " << oldval << std::endl;
      firstCwnd = false;
    }
  *cWndStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval << std::endl;
  cWndValue = newval;

  if (!firstSshThr)
    {
      *ssThreshStream->GetStream () << Simulator::Now ().GetSeconds () << " " << ssThreshValue << std::endl;
    }
}

static void
SsThreshTracer (uint32_t oldval, uint32_t newval)
{
  if (firstSshThr)
    {
      *ssThreshStream->GetStream () << "0.0 " << oldval << std::endl;
      firstSshThr = false;
    }
  *ssThreshStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval << std::endl;
  ssThreshValue = newval;

  if (!firstCwnd)
    {
      *cWndStream->GetStream () << Simulator::Now ().GetSeconds () << " " << cWndValue << std::endl;
    }
}

static void
RttTracer (Time oldval, Time newval)
{
  if (firstRtt)
    {
      //*rttStream->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
      firstRtt = false;
    }

    *rttStream->GetStream ()<<"(" << Simulator::Now ().GetSeconds () << ", " << newval.GetSeconds () <<")"<< std::endl;
  
  
}

static void
RtoTracer (Time oldval, Time newval)
{
  if (firstRto)
    {
      *rtoStream->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
      firstRto = false;
    }
  *rtoStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval.GetSeconds () << std::endl;
}
static void
RttVarTracer (Time oldval, Time newval)
{
  if (firstRttVar)
    {
      *rttvarStream->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
      firstRttVar = false;
    }
  *rttvarStream->GetStream () <<  Simulator::Now ().GetSeconds () << " " << newval.GetSeconds () << std::endl;
}




static void
RealRttTracer (Time oldval, Time newval)
{
  if (firstRealRtt)
    {
      //*rrttStream->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
      firstRealRtt = false;
    }

    *rrttStream->GetStream ()<<"("<< Simulator::Now ().GetSeconds () << ", " << newval.GetSeconds () <<")"<< std::endl;
}
  


static void
DeltaTracer (Time oldval, Time newval)
{
  if (firstDelta)
    {
      *deltaStream->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
      firstDelta = false;
    }
  *deltaStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval.GetSeconds () << std::endl;
}



/*****************************************************************************/

/*****************************************************************************/
/*SETTING UP FILES FOR STORING TRACE VALUES                                  */
/*****************************************************************************/
static void
TraceCwnd (std::string cwnd_tr_file_name)
{
  AsciiTraceHelper ascii;
  cWndStream = ascii.CreateFileStream (cwnd_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&CwndTracer));
}

static void
TraceSsThresh (std::string ssthresh_tr_file_name)
{
  AsciiTraceHelper ascii;
  ssThreshStream = ascii.CreateFileStream (ssthresh_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/SlowStartThreshold", MakeCallback (&SsThreshTracer));
}

static void
TraceRtt (std::string rtt_tr_file_name)
{
  AsciiTraceHelper ascii;
  rttStream = ascii.CreateFileStream (rtt_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/2/$ns3::TcpL4Protocol/SocketList/0/RTT", MakeCallback (&RttTracer));
}

static void
TraceRto (std::string rto_tr_file_name)
{
  AsciiTraceHelper ascii;
  rtoStream = ascii.CreateFileStream (rto_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/2/$ns3::TcpL4Protocol/SocketList/0/RTO", MakeCallback (&RtoTracer));
}
static void
TraceRealRtt (std::string rrtt_tr_file_name)
{
  AsciiTraceHelper ascii;
  rrttStream = ascii.CreateFileStream (rrtt_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/2/$ns3::TcpL4Protocol/SocketList/0/realRTT", MakeCallback (&RealRttTracer));
}
static void
TraceRttVar (std::string rttvar_tr_file_name)
{
  AsciiTraceHelper ascii;
  rttvarStream = ascii.CreateFileStream (rttvar_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/2/$ns3::TcpL4Protocol/SocketList/0/RTTvar", MakeCallback (&RttVarTracer));
}
static void
TraceDelta (std::string delta_tr_file_name)
{
  AsciiTraceHelper ascii;
  deltaStream = ascii.CreateFileStream (delta_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/3/$ns3::TcpL4Protocol/SocketList/*/Delta", MakeCallback (&DeltaTracer));
}




/**********EXPERIMENTAL SECOND RTT TRACE***********************************/

static void
RttTracer_2 (Time oldval, Time newval)
{
  if (firstRtt)
    {
      *rttStream_2->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
      firstRtt_2 = false;
    }
  *rttStream_2->GetStream () << Simulator::Now ().GetSeconds () << "," << newval.GetSeconds () << std::endl;
}


static void
TraceRtt_2 (std::string rtt_tr_file_name_2)
{
  AsciiTraceHelper ascii;
  rttStream_2 = ascii.CreateFileStream (rtt_tr_file_name_2.c_str ());
  Config::ConnectWithoutContext ("/NodeList/3/$ns3::TcpL4Protocol/SocketList/*/RTT", MakeCallback (&RttTracer_2));
}

/**********************************************/



int main (int argc, char *argv[])
{

  // User may find it convenient to enable logging
  LogComponentEnable("DumbbellDifferingFlows", LOG_LEVEL_ALL);
  LogComponentEnable("BulkSendApplication", LOG_LEVEL_INFO);
  //LogComponentEnable("RttEstimator", LOG_LEVEL_INFO);
  //LogComponentEnable("DropTailQueue", LOG_LEVEL_ALL);
  LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);


  std::string transport_prot = "TcpWestwood";

  double error_p = 0.0 ;

  /*Default TCP flavor*/
  NS_LOG_INFO ("Bottleneck Link: 10Mbps , 35ms delay");
  std::string shared_bandwidth = "10Mbps";
  std::string shared_delay = "35ms";

  NS_LOG_INFO ("Access Links: 2Mbps , 175ms delay");
  std::string access_bandwidth = "2Mbps";
  std::string access_delay = "175ms";


  NS_LOG_INFO ("Access2 Links: 2Mbps , 45ms delay");
  std::string access_bandwidth2 = "2Mbps";
  std::string access_delay2 = "45ms";



  /*Tracing, trace file names*/
  bool tracing = true;
  std::string tr_file_name = "";
  std::string cwnd_tr_file_name = "";
  std::string ssthresh_tr_file_name = "";
  std::string rtt_tr_file_name = "rtt_1.trace";
  std::string rto_tr_file_name = "";
  std::string rttvar_tr_file_name = "";
  std::string rrtt_tr_file_name = "rtt_real.trace";
  std::string delta_tr_file_name = "";

  std::string rtt_tr_file_name_2 = "rtt_2.trace";


  /*Simulation parameters*/

  /*MAX data is split accross all nodes
    MUST SET SUFFICIENTLY LARGE OTHERWISE
    SOME NODE WILL END TRANSMISSION EARLY*/
  //double data_mbytes = 2000000; //Number of Megabytes of data to transmit
  //uint32_t mtu_bytes = 20000; //Size of IP packets to send in bytes

  uint16_t num_nodes = 20;
  uint16_t num_flows = 5;
  float duration = 50;
  uint32_t run = 0;
  bool flow_monitor = true;

  SeedManager::SetSeed (1);
  SeedManager::SetRun (run);

  

  std::string dataRateString = "4Mbps";


  double range = 15.0;


  CommandLine cmd;
  cmd.AddValue ("transport_prot", "Transport protocol to use: TcpTahoe, TcpReno, TcpNewReno, TcpWestwood, TcpWestwoodPlus ", transport_prot);
  cmd.AddValue ("error_p", "Packet error rate", error_p);
  cmd.AddValue ("shared_bandwidth", "Bottleneck bandwidth", shared_bandwidth);
  cmd.AddValue ("access_bandwidth", "Access link bandwidth", access_bandwidth);
  cmd.AddValue ("delay", "Access link delay", access_delay);
  cmd.AddValue ("tracing", "Flag to enable/disable tracing", tracing);
  cmd.AddValue ("tr_name", "Name of output trace file", tr_file_name);
  cmd.AddValue ("cwnd_tr_name", "Name of output trace file", cwnd_tr_file_name);
  cmd.AddValue ("ssthresh_tr_name", "Name of output trace file", ssthresh_tr_file_name);
  cmd.AddValue ("rtt_tr_name", "Name of output trace file for ESIMATED RTT", rtt_tr_file_name);
  cmd.AddValue ("rto_tr_name", "Name of output trace file", rto_tr_file_name);
  cmd.AddValue ("rttvar_tr_name", "Name of output trace file", rttvar_tr_file_name);
  cmd.AddValue ("rrtt_tr_name", "Name of output trace file for REAL RTT", rrtt_tr_file_name);
  cmd.AddValue ("delta_tr_name", "Name of output trace file", delta_tr_file_name);
  //cmd.AddValue ("data", "Number of Megabytes of data to transmit", data_mbytes);
  //cmd.AddValue ("mtu", "Size of IP packets to send in bytes", mtu_bytes);
  cmd.AddValue ("num_flows", "Number of flows", num_flows);
  cmd.AddValue ("num_nodes", "Number of nodes", num_nodes);
  cmd.AddValue ("duration", "Time to allow flows to run in seconds", duration);
  cmd.AddValue ("run", "Run index (for setting repeatable seeds)", run);
  cmd.AddValue ("flow_monitor", "Enable flow monitor", flow_monitor);
  cmd.AddValue ("dataRateString", "String for data rate", dataRateString);
  cmd.AddValue ("range", "Coverage range", range);

  cmd.Parse (argc, argv);


  // Set the simulation start and stop time
  float start_time = 0.1;
  float stop_time = start_time + duration;



  // Calculate the ADU size
  /*Header* temp_header = new Ipv4Header ();
  uint32_t ip_header = temp_header->GetSerializedSize ();
  delete temp_header;


  temp_header = new TcpHeader ();
  uint32_t tcp_header = temp_header->GetSerializedSize ();
  delete temp_header;

  uint32_t tcp_adu_size = mtu_bytes - (ip_header + tcp_header);

  NS_LOG_INFO(tcp_adu_size);*/



  //Config::SetDefault ("ns3::RangePropagationLossModel::MaxRange", DoubleValue (50.0));

  // Select TCP variant
  if (transport_prot.compare ("TcpTahoe") == 0)
    {
      //Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpTahoe::GetTypeId ()));
    }
  else if (transport_prot.compare ("TcpReno") == 0)
    {
      //Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpReno::GetTypeId ()));
    }
  else if (transport_prot.compare ("TcpNewReno") == 0)
    {
      Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpNewReno::GetTypeId ()));
    }
  else if (transport_prot.compare ("TcpWestwood") == 0)
    { // the default protocol type in ns3::TcpWestwood is WESTWOOD
      Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpWestwood::GetTypeId ()));
      Config::SetDefault ("ns3::TcpWestwood::FilterType", EnumValue (TcpWestwood::TUSTIN));
    }
  else if (transport_prot.compare ("TcpWestwoodPlus") == 0)
    {
      Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpWestwood::GetTypeId ()));
      Config::SetDefault ("ns3::TcpWestwood::ProtocolType", EnumValue (TcpWestwood::WESTWOODPLUS));
      Config::SetDefault ("ns3::TcpWestwood::FilterType", EnumValue (TcpWestwood::TUSTIN));
    }
  else
    {
      NS_LOG_DEBUG ("Invalid TCP version");
      exit (1);
    }




    //int nodes_on_each_side = num_nodes/2;

    NodeContainer nodes;
    nodes.Create(num_nodes);

    // NodeContainer sources;

    // for(int i=0;i<nodes_on_each_side;i++){
    //   sources.Add(nodes.Get(i));
    // }

    // NodeContainer sinks;
    // for(int i=nodes_on_each_side;i<num_nodes;i++){
    //   sinks.Add(nodes.Get(i));
    // }

    // NodeContainer sources;
    // sources.Create(nodes_on_each_side);

    // NodeContainer sinks;
    // sinks.Create(nodes_on_each_side);


    // Create gateways, sources, and sinks
    /*NodeContainer leftGate;
    leftGate.Create (1);
    NS_LOG_INFO ("Number of Nodes After left Gate: " << NodeList::GetNNodes());


    NodeContainer rightGate;
    rightGate.Create (1);
    NS_LOG_INFO ("Number of Nodes After Right Gate: " << NodeList::GetNNodes());

    NodeContainer sources;
    sources.Create (nodes_on_each_side);

    NS_LOG_INFO ("Total Number of Nodes After Sources: " << NodeList::GetNNodes());

    NodeContainer sinks;
    sinks.Create (nodes_on_each_side);*/

    /*NS_LOG_INFO ("Total Number of Flows: " << num_flows);
    NS_LOG_INFO ("Total Number of Nodes After Sinks: " << NodeList::GetNNodes());
    Ptr< Node > tmp = sources.Get (0);
    NS_LOG_INFO ("First Source Node Index: " << tmp->GetId());
    tmp = sinks.Get (0);
    NS_LOG_INFO ("First Sink Node Index: " << tmp->GetId());
    NS_LOG_INFO ("TCP Flavor: " << transport_prot);*/


    // Configure the error model
    // Here we use RateErrorModel with packet error rate
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
    uv->SetStream (50);
    //Ptr<RateErrorModel> error_model = CreateObject<RateErrorModel> ();
    RateErrorModel error_model ;
    error_model.SetRandomVariable (uv);
    error_model.SetUnit (RateErrorModel::ERROR_UNIT_PACKET);
    error_model.SetRate (error_p);


    /*

    //create access points/gateways
    PointToPointHelper BottleNeckLink;
    BottleNeckLink.SetDeviceAttribute ("DataRate", StringValue (shared_bandwidth));
    BottleNeckLink.SetChannelAttribute ("Delay", StringValue (shared_delay));
    BottleNeckLink.SetDeviceAttribute ("ReceiveErrorModel", PointerValue (&error_model));


    //net device container for the gates
    NetDeviceContainer gates;
    gates = BottleNeckLink.Install (leftGate.Get (0),rightGate.Get (0));

    */
    
    //creating two wifi networks


    //wifi channel helpers for the two wifi networks
    //YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss ("ns3::RangePropagationLossModel");


    //YansWifiChannelHelper channel1 = YansWifiChannelHelper::Default();
    // YansWifiChannelHelper channel1;
    // channel1.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // channel1.AddPropagationLoss ("ns3::RangePropagationLossModel");

    //wifi physical helpers
    YansWifiPhyHelper phy;
    //YansWifiPhyHelper phy1;

    phy.SetChannel(channel.Create());
    //phy1.SetChannel(channel1.Create());

    //phy.Set ("TxPowerStart",DoubleValue (txp));
    //phy.Set ("TxPowerEnd", DoubleValue (txp));

    //wifi helper
    WifiHelper wifi;
    //wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager");
    //wifi.SetStandard(WIFI_STANDARD_80211b);



    //setting up wifi mac helpers for the two networks
    WifiMacHelper mac;
    /*Ssid ssid = Ssid ("ns-3-ssid");
    mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));*/

    mac.SetType ("ns3::AdhocWifiMac");

    //install wifi for sources and sinks
    NetDeviceContainer devices;
    devices= wifi.Install(phy, mac, nodes);
    //rightDevices = wifi.Install(phy1, mac, sinks);


    //set mac for access poimts
    /*mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));
    */

    //setting up wifi for access points
    /*NetDeviceContainer leftDevices,rightDevices;
    apDevicesLeft = wifi.Install (phy, mac, leftGate);
    apDevicesRight = wifi.Install (phy1, mac, rightGate);*/


    /*
    //set up mobility models
    MobilityHelper mobility1;
    MobilityHelper mobility2;
    //MobilityHelper mobility3;



    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=25]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    
    mobility1.SetPositionAllocator (taPositionAlloc);

    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=26|Max=50]"));

    taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();

    mobility2.SetPositionAllocator (taPositionAlloc);

    mobility1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility2.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    //mobility3.SetMobilityModel ("ns3::ConstantPositionMobilityModel");



    mobility1.Install (sources);
    mobility2.Install (sinks);
    mobility1.Install (leftGate);
    mobility2.Install (rightGate);
    */

   MobilityHelper mobility;

  
   std::ostringstream cvg;
   cvg << "ns3::UniformRandomVariable[Min=0.0|Max="<<range<<"]";
   std::string rect (cvg.str());
   //std::cout<<"rectange "<<rect<<" by "<<rect<<std::endl;

  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue (rect));
  pos.Set ("Y", StringValue (rect));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();

  /* mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (0.5),
                                 "DeltaY", DoubleValue (1.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));*/

  //mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
  //                           "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(taPositionAlloc);

  mobility.Install (nodes);
  //mobility.Install (sinks);
  //mobility.Install (leftGate);
  //mobility.Install (rightGate);

  

  


  LrWpanHelper lrWpanHelper1;
  // Add and install the LrWpanNetDevice for each node
  // lrWpanHelper.EnableLogComponents();
  NetDeviceContainer devContainer = lrWpanHelper1.Install(nodes);
  lrWpanHelper1.AssociateToPan (devContainer, 10);

  // LrWpanHelper lrWpanHelper2;
  // NetDeviceContainer devContainerSinks = lrWpanHelper2.Install(sinks);
  // lrWpanHelper2.AssociateToPan (devContainerSinks, 10);
  
  /*LrWpanHelper lrWpanHelper3;
  NetDeviceContainer devContainerLeftGate = lrWpanHelper3.Install(leftGate.Get(0));
  lrWpanHelper3.AssociateToPan (devContainerLeftGate, 10);

  LrWpanHelper lrWpanHelper4;
  NetDeviceContainer devContainerRightGate = lrWpanHelper4.Install(rightGate.Get(0));
  lrWpanHelper4.AssociateToPan (devContainerRightGate, 10);

    NodeContainer const & n = NodeContainer::GetGlobal ();
    for (NodeContainer::Iterator i = n.Begin (); i != n.End (); ++i)
    {
        Ptr<Node> node = *i;
        std::string name = Names::FindName (node);

        Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
        if(!mob) continue;

        Vector pos = mob->GetPosition ();
        std::cout << "Node " << name << " is at (" << pos.x << ", " <<
        pos.y << ", " << pos.z << ")\n";

    }



    
    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    
    mobilityAdhoc.SetPositionAllocator (taPositionAlloc);

    
    
    */



    //install internet on the nodes

    AodvHelper aodv;
    InternetStackHelper internetv6;
    internetv6.SetRoutingHelper (aodv);
    //internet.Install(sources);
    //internet.Install(sinks);


    //InternetStackHelper internetv6;
    internetv6.SetIpv4StackInstall (false);
    internetv6.Install (nodes);
    //internetv6.Install (sinks);
    //internetv6.Install (leftGate);
    //internetv6.Install (rightGate);


    SixLowPanHelper sixlowpan1;
    NetDeviceContainer six1 = sixlowpan1.Install (devices);

    //SixLowPanHelper sixlowpan2;
    //NetDeviceContainer six2 = sixlowpan2.Install (devContainerSinks);
/*
    SixLowPanHelper sixlowpan3;
    NetDeviceContainer six3 = sixlowpan3.Install (devContainerLeftGate);

    SixLowPanHelper sixlowpan4;
    NetDeviceContainer six4 = sixlowpan4.Install (devContainerRightGate);

    NetDeviceContainer container;
    container.Add(six3);
    container.Add(six4);
*/
    //addressing the devices
    Ipv6AddressHelper ipv6;

    //addressing the backbone p2p network
    ipv6.SetBase (Ipv6Address ("2001:1::"), Ipv6Prefix (64));
    // Ipv6InterfaceContainer left_device = ipv6.Assign (container);
    // NS_LOG_INFO("left gate address: "<<left_device.GetAddress(0,1) );

    //ipv6.;
    //Ipv6InterfaceContainer right_device = ipv6.Assign (six4);
    //NS_LOG_INFO("right gate address: "<<right_device.GetAddress(0,1) );




    //address.SetBase ("10.1.1.0", "255.255.255.0");

    //ipv6.NewNetwork();
    Ipv6InterfaceContainer node_interfaces = ipv6.Assign (six1);
    //Ipv6InterfaceContainer source_ap = ipv6.Assign (six3);

    //ipv6.NewNetwork();
    //Ipv6InterfaceContainer sink_interfaces = ipv6.Assign (six2);
    //Ipv6InterfaceContainer sink_ap = ipv6.Assign (six4);

    //Ipv4InterfaceContainer backbone = address.Assign(gates);

    /*NS_LOG_INFO("Backbones: \n");
    for(uint32_t i=0;i<backbone.GetN();i++){
      NS_LOG_INFO(backbone.GetAddress(i)<<"\n");
    }*/

    //addressing the sources
    //address.SetBase ("10.1.2.0", "255.255.255.0");
    
    //Ipv4InterfaceContainer source_interfaces;
    //source_interfaces=address.Assign(leftDevices);

    /*NS_LOG_INFO("Left Devices: \n");
    for(uint32_t i=0;i<source_interfaces.GetN();i++){
      NS_LOG_INFO(source_interfaces.GetAddress(i)<<"\n");
    }*/

    //addressing the left access point
    //Ipv4InterfaceContainer source_ap=address.Assign(apDevicesLeft);
    
    //NS_LOG_INFO("Left Device Access Point: "<<source_ap.GetAddress(0)<<"\n");



    //addressing the sinks
    //address.SetBase ("10.1.3.0", "255.255.255.0");

    //Ipv4InterfaceContainer sink_interfaces=address.Assign(rightDevices);
    //NS_LOG_INFO("Sinks Assigned");


    /*NS_LOG_INFO("Right Devices: \n");
    for(uint32_t i=0;i<sink_interfaces.GetN();i++){
      NS_LOG_INFO(sink_interfaces.GetAddress(i)<<"\n");
    }*/


    //addressing the right access point
    //Ipv4InterfaceContainer sink_ap=address.Assign(apDevicesRight);
    //NS_LOG_INFO("Sink Access Point Assigned");
    //NS_LOG_INFO("Right Device Access Point: "<<sink_ap.GetAddress(0)<<"\n");
    

    // Configure the sources and sinks net devices
    // and the channels between the sources/sinks and the gateways
    //PointToPointHelper LocalLinkL;
    //PointToPointHelper LocalLinkR;

    /*

    LocalLinkL.SetDeviceAttribute ("DataRate", StringValue (access_bandwidth));
    LocalLinkL.SetChannelAttribute ("Delay", StringValue (access_delay));
    LocalLinkR.SetDeviceAttribute ("DataRate", StringValue (access_bandwidth));
    LocalLinkR.SetChannelAttribute ("Delay", StringValue (access_delay));
    Ipv4InterfaceContainer sink_interfaces;
    */

    /*for (int i = 0; i < num_flows; i++)
      {

        if ((i%2) == 0) {

          LocalLinkL.SetDeviceAttribute ("DataRate", StringValue (access_bandwidth));
          LocalLinkL.SetChannelAttribute ("Delay", StringValue (access_delay));
          LocalLinkR.SetDeviceAttribute ("DataRate", StringValue (access_bandwidth));
          LocalLinkR.SetChannelAttribute ("Delay", StringValue (access_delay));



          NetDeviceContainer devices;
          devices = LocalLinkL.Install (sources.Get (i), leftGate.Get (0));

          address.NewNetwork ();
          Ipv4InterfaceContainer interfaces = address.Assign (devices);


          devices = LocalLinkR.Install (rightGate.Get (0), sinks.Get (i));
          address.NewNetwork ();
          interfaces = address.Assign (devices);
          sink_interfaces.Add (interfaces.Get (1));


        } else {
          LocalLinkL.SetDeviceAttribute ("DataRate", StringValue (access_bandwidth2));
          LocalLinkL.SetChannelAttribute ("Delay", StringValue (access_delay2));
          LocalLinkR.SetDeviceAttribute ("DataRate", StringValue (access_bandwidth2));
          LocalLinkR.SetChannelAttribute ("Delay", StringValue (access_delay2));



          NetDeviceContainer devices;
          devices = LocalLinkL.Install (sources.Get (i), leftGate.Get (0));

          address.NewNetwork ();
          Ipv4InterfaceContainer interfaces = address.Assign (devices);


          devices = LocalLinkR.Install (rightGate.Get (0), sinks.Get (i));
          address.NewNetwork ();
          interfaces = address.Assign (devices);
          sink_interfaces.Add (interfaces.Get (1));
        }

      }*/




      //populate routing tables
      //NS_LOG_INFO ("Initialize Global Routing.");
      //Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
      //
      

      uint16_t port = 9;


      /*Address sinkLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);
      */

      //Address sinkLocalAddress (InetSocketAddress (sink_interfaces.GetAddress(0), port));
      //PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);      //NS_LOG_INFO ("Number of Sources = " + sources.GetN());

      Address sinkLocalAddress (Inet6SocketAddress(Ipv6Address::GetAny(), port));
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);

      
      //std::cout<<"Number of sources = "<<sources.GetN()<<" Sink interface size = "<< sink_interfaces.GetN() <<"\n";

      for (uint16_t i = 0; i < num_flows; i++)
        { 
          //Address sinkLocalAddress (InetSocketAddress (sink_interfaces.GetAddress(i), port));
          //PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);

          AddressValue remoteAddress (Inet6SocketAddress(node_interfaces.GetAddress (num_flows+i,1), port));
          //std::cout<<"Current Port = "<<port<<"\n";

          if (transport_prot.compare ("TcpTahoe") == 0
              || transport_prot.compare ("TcpReno") == 0
              || transport_prot.compare ("TcpNewReno") == 0
              || transport_prot.compare ("TcpWestwood") == 0
              || transport_prot.compare ("TcpWestwoodPlus") == 0)
            {
              //Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (tcp_adu_size));
              Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1448));
              //UintegerValue (1448)
              
              OnOffHelper ftp ("ns3::TcpSocketFactory", Address ());

              ftp.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
              ftp.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
              ftp.SetAttribute ("DataRate", DataRateValue (DataRate (dataRateString)));
              ftp.SetAttribute ("PacketSize", UintegerValue (512));
              ftp.SetAttribute ("Remote", remoteAddress);

              /*BulkSendHelper ftp ("ns3::TcpSocketFactory", Address ());

              ftp.SetAttribute ("Remote", remoteAddress);
              ftp.SetAttribute ("SendSize", UintegerValue (tcp_adu_size));
              ftp.SetAttribute ("MaxBytes", UintegerValue (int(data_mbytes * 1000000)));*/

              ApplicationContainer sourceApp = ftp.Install (nodes.Get (i));
              sourceApp.Start (Seconds (start_time * i));
              
              //NS_LOG_INFO("Source app started");

              sourceApp.Stop (Seconds (stop_time - 3));

              //NS_LOG_INFO("Source app started");

              sinkHelper.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
              ApplicationContainer sinkApp = sinkHelper.Install (nodes.Get(num_flows+i));
              sinkApp.Start (Seconds (start_time * i));

              //NS_LOG_INFO("Sink app started");

              sinkApp.Stop (Seconds (stop_time));
            }
          else
            {
              NS_LOG_DEBUG ("Invalid transport protocol " << transport_prot << " specified");
              exit (1);
            }

            //port++;
        }
        
        //std::cout<<"Reached here\n";


          // Set up tracing if enabled
          if (tracing)
            {

              
              double trace_start=0.5;
              if (tr_file_name.compare ("") != 0)
                {
                  std::ofstream ascii;
                  Ptr<OutputStreamWrapper> ascii_wrap;
                  ascii.open (tr_file_name.c_str ());
                  ascii_wrap = new OutputStreamWrapper (tr_file_name.c_str (), std::ios::out);
                  internetv6.EnableAsciiIpv6All (ascii_wrap);
                }

              if (cwnd_tr_file_name.compare ("") != 0)
                {
                  Simulator::Schedule (Seconds (trace_start), &TraceCwnd, cwnd_tr_file_name);
                }

              if (ssthresh_tr_file_name.compare ("") != 0)
                {
                  Simulator::Schedule (Seconds (trace_start), &TraceSsThresh, ssthresh_tr_file_name);
                }
              /*****estimate RTT**************************************************************/
              if (rtt_tr_file_name.compare ("") != 0)
                {
                  Simulator::Schedule (Seconds (trace_start), &TraceRtt, rtt_tr_file_name);
                }

              if (rto_tr_file_name.compare ("") != 0)
                {
                  Simulator::Schedule (Seconds (trace_start), &TraceRto, rto_tr_file_name);
                }

  		        /*****RTT VARIANCE**************************************************************/
              if (rttvar_tr_file_name.compare ("") != 0)
                {
                  Simulator::Schedule (Seconds (trace_start), &TraceRttVar, rttvar_tr_file_name);
                }
              /*****REAL RTT**************************************************************/
              if (rrtt_tr_file_name.compare ("") != 0)
                {
                  Simulator::Schedule (Seconds (trace_start), &TraceRealRtt, rrtt_tr_file_name);
                }

                /*****Delta**************************************************************/
              if (delta_tr_file_name.compare ("") != 0)
                {
                  Simulator::Schedule (Seconds (trace_start), &TraceDelta, delta_tr_file_name);
                }

                /*****estimate RTT**************************************************************/
                if (rtt_tr_file_name_2.compare ("") != 0)
                  {
                    Simulator::Schedule (Seconds (trace_start), &TraceRtt_2, rtt_tr_file_name_2);
                  }

            }

            //std::cout<<"Reached Here as well\n";

          //BottleNeckLink.EnablePcapAll ("DumbbellDifferingFlows", true);
          //LocalLinkL.EnablePcapAll ("DumbbellDifferingFlowsSource", true);
          //LocalLinkR.EnablePcapAll ("DumbbellDifferingFlowsDestination", true);

          // Flow monitor
          FlowMonitorHelper flowHelper;
          if (flow_monitor)
            {
              flowHelper.InstallAll ();
            }

          Simulator::Stop (Seconds (stop_time));

          std::cout<<"Reached after stop\n";

          int rangeInt = (int)range;


          Simulator::Run ();

          //std::string xmlName = "1705038_taskA_nodes" + num_nodes + "_flows" + num_flows + ".flowmonitor" ;
          //std::strcat(xmlName, num_nodes);
          //std::str

          std::ostringstream xmlName;
          xmlName<<"1705038_taskA_802_15_nodes"<< num_nodes<< "_flows"<< num_flows <<"_datarate"<<dataRateString<<
           "_range"<<rangeInt<<".flowmonitor";
          std::string fileName(xmlName.str());
          /*
          
          std::ostringstream dr;
          dr << datarate<<"bps";
          std::string rate (dr.str());
          
          */

          //xmlName.append(num_nodes);

          //std::cout<<"Reached after run\n";

          if (flow_monitor)
            { 
              flowHelper.SerializeToXmlFile (fileName, true, true);
            }

          Simulator::Destroy ();
          return 0;
        }
