/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/core-module.h"
#include "experiments/experiment.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Main");


void RunTest() {
}

int main (int argc, char *argv[])
{
  Config::SetDefault("ns3::YansWifiPhy::TxPowerStart", DoubleValue(0));
  Config::SetDefault("ns3::YansWifiPhy::TxPowerEnd", DoubleValue(19));//44.8
  Config::SetDefault("ns3::YansWifiPhy::TxPowerLevels", UintegerValue(10));
  
  LogComponentEnable("Main", LOG_LEVEL_ALL);
  LogComponentEnable("Experiment", LOG_LEVEL_ALL);

  std::string expName = "Experiment";
  std::string esmpName = "EsmProtocol";
  std::string rsmpName = "RsmProtocol";
  CommandLine cmd;
  cmd.AddValue("exp", "Experiment scenario", expName);
  cmd.AddValue("ep", "ESM Protocol", esmpName);
  cmd.AddValue("rp", "RSM Protocol", rsmpName);
  //std::cout << argv[0] <<" " << argv[1] << " " << argv[2] << std::endl;
  cmd.Parse (argc, argv);
  if (expName == "Test") {
    RunTest();
  } else {
    ObjectFactory factory;
    factory.SetTypeId(expName);
    Ptr<Experiment> exp = factory.Create<Experiment>();
    exp->SetEsmProtocol(esmpName);
    exp->SetRsmProtocol(rsmpName);
    exp->Run();
    //exp = 0;
  }
  //std::cout << expName << std::endl;
  return 0;
}
