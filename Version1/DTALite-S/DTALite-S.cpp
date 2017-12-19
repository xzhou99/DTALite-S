// AgentPlus.cpp : Defines the entry point for the console application./* Copyright (C) 2017 Dr. Xuesong Zhou - All Rights Reserved*/

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <list> 
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "CSVParser.h"
#include <functional>
#include<stdio.h>   
#include<tchar.h>

#define _MAX_LABEL_COST 99999
#define _MAX_NUMBER_OF_PAX 100
#define _MAX_NUMBER_OF_VEHICLES 100
#define _MAX_NUMBER_OF_TIME_INTERVALS 100
#define _MAX_NUMBER_OF_NODES 100

// Linear congruential generator 
#define LCG_a 17364
#define LCG_c 0
#define LCG_M 65521  // it should be 2^32, but we use a small 16-bit number to save memory


// The one and only application object

CWinApp theApp;
using namespace std;

TCHAR g_SettingFileName[_MAX_PATH] = _T("./Settings.txt");

FILE* g_pFileDebugLog = NULL;

FILE* g_pFileOutputLog = NULL;

int g_number_of_threads = 4;
int g_shortest_path_debugging_flag = 0;
int g_number_of_agents;

#define _MAX_ZONE_SIZE 4000
double g_loading_multiplier = 0.66666666;
double g_ODME_adjusment_step_size = 0.01;

double g_OD_loading_multiplier[_MAX_ZONE_SIZE][_MAX_ZONE_SIZE];

int g_number_of_simulation_intervals = 1200;  // 3600 per seconds  
int g_number_of_seconds_per_interval = 6;
int g_Simulation_StartTimeInMin = 9999;
int g_Simulation_EndTimeInMin = 0;



// 6 seconds per interval
// 3600 -> 6
// 1800 -> 3
// 900 -> 1.5

std::map<int, int> g_link_key_to_seq_no_map;  // hush table, map key to internal link sequence no. 

long g_GetLinkSeqNo(int from_node_seq_no, int to_node_seq_no)
{
	long link_key = from_node_seq_no * 100000 + to_node_seq_no;

	if (g_link_key_to_seq_no_map.find(link_key) != g_link_key_to_seq_no_map.end())
		return g_link_key_to_seq_no_map[link_key];
	else
		return -1;
}

//mfd
int g_TAU;


std::map<int, int> g_internal_node_seq_no_map;  // hush table, map external node number to internal node sequence no. 

class CLink
{
public: 
	CLink()  // construction 
	{
		cost = 0;
		BRP_alpha = 0.15f;
		BRP_beta = 4.0f;
		link_capacity = 1000;
		free_flow_travel_time_in_min = 1;
		flow_volume = 0;
		// mfd
		mfd_zone_id = 0;

		m_LinkOutFlowCapacity = NULL;
		m_LinkInFlowCapacity = NULL;
		m_LinkCumulativeArrival = NULL;
		m_LinkCumulativeDeparture = NULL;
		m_LinkTravelTime = NULL;

		m_LinkObsFlow = NULL; 
		m_LinkObsDensity = NULL;
		m_LinkObsTravelTime = NULL;
		m_LinkObsFlowDeviation = NULL;
		m_LinkObsDensityDeviation = NULL;
		m_LinkObsMarginal = NULL;

	}

	~CLink()
	{
		DeallocateMemory();
	}

	int* m_LinkOutFlowCapacity;
	int* m_LinkInFlowCapacity;
	int* m_LinkCumulativeArrival;
	int* m_LinkCumulativeDeparture;

	float* m_LinkObsFlow;
	float* m_LinkObsDensity;   //
	float* m_LinkObsTravelTime;  // in min
	float* m_LinkObsFlowDeviation;
	float* m_LinkObsDensityDeviation;   //
	float* m_LinkObsMarginal;

	float* m_LinkTravelTime;

	int GetRandommizeCapacity(float link_capacity)
	{
		return int(link_capacity*g_number_of_seconds_per_interval / 3600 + 0.5);
	}

	void AllocateMemory()
	{
		m_LinkOutFlowCapacity = new int[g_number_of_simulation_intervals];
		m_LinkInFlowCapacity = new int[g_number_of_simulation_intervals];
		m_LinkCumulativeArrival = new int[g_number_of_simulation_intervals];
		m_LinkCumulativeDeparture = new int[g_number_of_simulation_intervals];
		m_LinkTravelTime = new float[g_number_of_simulation_intervals];

		m_LinkObsFlow = new float[g_number_of_simulation_intervals];
		m_LinkObsDensity = new float[g_number_of_simulation_intervals];
		m_LinkObsTravelTime = new float[g_number_of_simulation_intervals];
		m_LinkObsFlowDeviation = new float[g_number_of_simulation_intervals];
		m_LinkObsDensityDeviation = new float[g_number_of_simulation_intervals];
		m_LinkObsMarginal = new float[g_number_of_simulation_intervals];

	

		for (int t = 0; t < g_number_of_simulation_intervals; t++)
		{
			m_LinkOutFlowCapacity[t] = GetRandommizeCapacity(link_capacity);
			m_LinkCumulativeArrival[t] = 0;
			m_LinkCumulativeDeparture[t] = 0;

			m_LinkObsDensity[t] = -1;
			m_LinkObsTravelTime[t] = -1;
			m_LinkObsFlowDeviation[t] = -1;
			m_LinkObsDensityDeviation[t] = -1;
			m_LinkObsMarginal[t] = 0;


		}

		free_flow_travel_time_in_simu_interval = int(free_flow_travel_time_in_min*60.0 / g_number_of_seconds_per_interval + 0.5);
	}

	void DeallocateMemory()
	{
		//if(m_LinkOutFlowCapacity != NULL) delete m_LinkOutFlowCapacity;
		//if (m_LinkInFlowCapacity != NULL) delete m_LinkInFlowCapacity;
		//if (m_LinkCumulativeArrival != NULL) delete m_LinkCumulativeArrival;
		//if (m_LinkCumulativeDeparture != NULL) delete m_LinkCumulativeDeparture;
		//if (m_LinkTravelTime != NULL) delete m_LinkTravelTime;

	}
	void UpdateMeasurementDeviation()
	{
		for (int t = 0; t < g_number_of_simulation_intervals; t++)
		{
			/// m_LinkObsFlowDeviation
			///m_LinkObsDensityDeviation 
			///m_LinkObsMarginal
		}
	
	}
	int link_seq_no;
	int from_node_seq_no;
	int to_node_seq_no;
	float cost;
	float free_flow_travel_time_in_min;
	int free_flow_travel_time_in_simu_interval;

	int type;
	float link_capacity;
	float flow_volume;
	float travel_time;
	float BRP_alpha;
	float BRP_beta;
	float length; 
	// mfd
	int mfd_zone_id;

	void CalculateBRPFunction()
	{
		travel_time = free_flow_travel_time_in_min*(1 + BRP_alpha*pow(flow_volume / max(0.00001, link_capacity), BRP_beta));
		cost = travel_time;
	}

	float get_VOC_ratio()
	{
		return flow_volume / max(0.00001, link_capacity);
	
	}

	float get_speed()
	{
		return length / max(travel_time, 0.0001) * 60;  // per hour
	}
	// mfd 



};


class CNode
{ 
public:
	CNode()
	{
		zone_id = 0;
		accessible_node_count = 0;
	}

	int accessible_node_count;

	int node_seq_no;  // sequence number 
	int node_id;      //external node number 
	int zone_id;
	double x;
	double y;

	std::vector<CLink> m_outgoing_node_vector;
	
};
std::vector<CNode> g_node_vector;
std::vector<CLink> g_link_vector;

class CAgent
{
public:

	unsigned int m_RandomSeed;
	bool m_bGenereated; 
	CAgent()
	{
		m_bGenereated = false;
		PCE_factor = 1.0;
		path_cost = 0;
		m_Veh_LinkArrivalTime = NULL;
		m_Veh_LinkDepartureTime = NULL;
		m_bCompleteTrip = false;
	}
	~CAgent()
	{
	
		DeallocateMemory();
	}

	float GetRandomRatio()
	{
		m_RandomSeed = (LCG_a * m_RandomSeed + LCG_c) % LCG_M;  //m_RandomSeed is automatically updated.

		return float(m_RandomSeed) / LCG_M;
	}

	void VehicleGeneration()
	{

		m_bGenereated = true;

		//float random_number = GetRandomRatio();  // between 0, 1 
		//if (random_number < g_OD_loading_multiplier[origin_zone_seq_no][destination_zone_seq_no])
		//	m_bGenereated = true;
		//else
		//	m_bGenereated = false;

	}
	int agent_id;
	int origin_node_id;
	int destination_node_id;

	int origin_zone_seq_no;
	int destination_zone_seq_no;

	float departure_time_in_min;
	int departure_time_in_simulation_interval;

	float PCE_factor;  // passenger car equivalent : bus = 3
	float path_cost; 
	std::vector<int> path_link_seq_no_vector;
	std::vector<int> path_node_seq_no_vector;

	int m_current_link_seq_no;
	int* m_Veh_LinkArrivalTime;
	int* m_Veh_LinkDepartureTime;

	//above are simulated 

	bool m_bCompleteTrip;

	void AllocateMemory()
	{
		if(m_Veh_LinkArrivalTime == NULL)
		{
		m_current_link_seq_no = 0;
		m_Veh_LinkArrivalTime = new int[path_link_seq_no_vector.size()];
		m_Veh_LinkDepartureTime = new int[path_link_seq_no_vector.size()];

		departure_time_in_simulation_interval = int(departure_time_in_min*60.0 / g_number_of_seconds_per_interval+0.5);

		if (path_link_seq_no_vector.size() > 0)
		{
			m_Veh_LinkArrivalTime[0] = departure_time_in_simulation_interval;

			int FirstLink = path_link_seq_no_vector[0];

			m_Veh_LinkDepartureTime[0] = m_Veh_LinkArrivalTime[0] + g_link_vector[FirstLink].free_flow_travel_time_in_simu_interval;
			g_link_vector[FirstLink].m_LinkCumulativeArrival[departure_time_in_simulation_interval] += 1;
		}
		}
	}

	void DeallocateMemory()
	{
		//if (m_Veh_LinkArrivalTime != NULL) delete m_Veh_LinkArrivalTime;
		//if (m_Veh_LinkDepartureTime != NULL) delete m_Veh_LinkDepartureTime;

	}

	void UpdateAgentMarginal()
	{
		float Marginal = 0;
		for (int i = 0; i < path_link_seq_no_vector.size(); i++)
		{
			int link_seq_no = path_link_seq_no_vector[i];
			int arrival_time = m_Veh_LinkArrivalTime[i];
			
			Marginal += g_link_vector[link_seq_no].m_LinkObsMarginal[arrival_time];

		}

		g_OD_loading_multiplier[origin_zone_seq_no][destination_zone_seq_no] += g_ODME_adjusment_step_size * Marginal;

	}
};


vector<CAgent> g_agent_vector;

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_zones = 0;


void g_ProgramStop()
{

	cout << "AgentLite Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};


vector<int> ParseLineToIntegers(string line)
{
	vector<int> SeperatedIntegers;
	string subStr;
	istringstream ss(line);


	char Delimiter = ';';


	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedIntegers.push_back(integer);
	}
	return SeperatedIntegers;
}

void g_ReadInputData()
{
	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0

	int internal_node_seq_no = 0;
	double x, y;
	// step 1: read node file 
	CCSVParser parser;
	if (parser.OpenCSVFile("input_node.csv", true))
	{
		std::map<int, int> node_id_map;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string name;

			int node_type;
			int node_id;

			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;

			if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
			{
				continue; //has been defined
			}
			g_internal_node_seq_no_map[node_id] = internal_node_seq_no;


			parser.GetValueByFieldName("x", x,false);
			parser.GetValueByFieldName("y", y, false);

		
			CNode node;  // create a node object 

			node.node_id = node_id;
			node.node_seq_no = internal_node_seq_no;
			parser.GetValueByFieldName("zone_id", node.zone_id);

			node.x = x;
			node.y = y;
			internal_node_seq_no++;

			g_node_vector.push_back(node);  // push it to the global node vector

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 == 0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}

		cout << "number of nodes = " << g_number_of_nodes << endl;

		fprintf(g_pFileOutputLog, "number of nodes =,%d\n", g_number_of_nodes);
		parser.CloseCSVFile();
	}

	// step 2: read link file 

	CCSVParser parser_link;

	if (parser_link.OpenCSVFile("input_link.csv", true))
	{
		while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			if (parser_link.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser_link.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;

				// add the to node id into the outbound (adjacent) node list

			int internal_from_node_seq_no = g_internal_node_seq_no_map[from_node_id];  // map external node number to internal node seq no. 
			int internal_to_node_seq_no = g_internal_node_seq_no_map[to_node_id];
			
			CLink link;  // create a link object 

			link.from_node_seq_no = internal_from_node_seq_no;
			link.to_node_seq_no = internal_to_node_seq_no;
			link.link_seq_no = g_number_of_links;
			link.to_node_seq_no = internal_to_node_seq_no;
	
			parser_link.GetValueByFieldName("link_type", link.type);

			float length =1; // km or mile
			float speed_limit = 1; 
			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("speed_limit", speed_limit);

				

			parser_link.GetValueByFieldName("BPR_alpha_term", link.BRP_alpha);
			parser_link.GetValueByFieldName("BPR_beta_term", link.BRP_beta);
			int number_of_lanes = 1;
			float lane_cap = 1000;
			parser_link.GetValueByFieldName("number_of_lanes", number_of_lanes);
			parser_link.GetValueByFieldName("lane_cap", lane_cap);

			link.link_capacity = lane_cap* number_of_lanes;

			link.free_flow_travel_time_in_min = length / speed_limit * 60;
			link.length = length;
			link.cost = length / speed_limit * 60; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate delay 


			g_node_vector[internal_from_node_seq_no].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link
			
			long link_key = internal_from_node_seq_no * 100000 + internal_to_node_seq_no;

			g_link_key_to_seq_no_map[link_key] = link.link_seq_no;
			g_link_vector.push_back(link);

			g_number_of_links++;

				if (g_number_of_links % 1000 == 0)
					cout << "reading " << g_number_of_links << " links.. " << endl;
			}
		}

		cout << "number of links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);

		parser_link.CloseCSVFile();

		g_number_of_agents = 0;
		CCSVParser parser_agent;
		std::vector<int> path_node_sequence;
		string path_node_sequence_str;

		if (parser_agent.OpenCSVFile("input_agent.csv", true))   // read agent as demand input 
		{
			while (parser_agent.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
			{
				CAgent agent;  // create an agent object 
				if (parser_agent.GetValueByFieldName("agent_id", agent.agent_id) == false)
					continue;

				agent.m_RandomSeed = agent.agent_id;

				int origin_node_id = 0;
				int destination_node_id = 0;
				parser_agent.GetValueByFieldName("from_origin_node_id", origin_node_id, false);

				agent.origin_node_id = origin_node_id;   
				parser_agent.GetValueByFieldName("to_destination_node_id", destination_node_id, false);
				agent.destination_node_id = destination_node_id;

				if (g_internal_node_seq_no_map.find(origin_node_id) == g_internal_node_seq_no_map.end() || g_internal_node_seq_no_map.find(destination_node_id) == g_internal_node_seq_no_map.end())
					continue;

				parser_agent.GetValueByFieldName("origin_zone_seq_no", agent.origin_zone_seq_no, false);
				parser_agent.GetValueByFieldName("destination_zone_seq_no", agent.destination_zone_seq_no, false);

				
				

				parser_agent.GetValueByFieldName("departure_time_in_min", agent.departure_time_in_min);

				if (agent.departure_time_in_min < g_Simulation_StartTimeInMin)
					g_Simulation_StartTimeInMin = agent.departure_time_in_min;

				if (agent.departure_time_in_min > g_Simulation_EndTimeInMin)
					g_Simulation_EndTimeInMin = agent.departure_time_in_min;
				
				parser_agent.GetValueByFieldName("PCE", agent.PCE_factor);
				parser_agent.GetValueByFieldName("path_node_sequence", path_node_sequence_str);

				agent.path_node_seq_no_vector = ParseLineToIntegers(path_node_sequence_str);
							

				if(agent.path_node_seq_no_vector.size() >=2)
				{
				for (int n = 0; n < agent.path_node_seq_no_vector.size() - 1; n++)
				{
					int link_seq_no = g_GetLinkSeqNo(agent.path_node_seq_no_vector[n], agent.path_node_seq_no_vector[n+1]);
					agent.path_link_seq_no_vector.push_back(link_seq_no);
				}

				}
				//To Do 1: load agent path from field path_node_sequence
				// initial loading multiplier: 0.66666

				g_agent_vector.push_back(agent);
				g_number_of_agents++;
				if(g_number_of_agents%1000==0)
					cout << "reading = " << g_number_of_agents/1000 << " k agents..." << endl;

			}
		}
		cout << "number of agents = " << g_agent_vector.size() << endl;

		g_number_of_simulation_intervals = (300)*60/g_number_of_seconds_per_interval;
		parser_agent.CloseCSVFile();


		//To Do 2: load input_sensor_data // speed, link volume, link density, per k min: starting time, ending time, from node, to node
		//To Do 3: load input_senario  // for prediction outflow_capacity, predicted free-flow speed
//


}

class CNode2NodeAccessibility
{
public:
	int from_node_no;
	int to_node_no;
	float travel_cost;
};

class NetworkForSP  // mainly for shortest path calculation
{
public:
	int m_threadNo;  // internal thread number 

    std::list<int>  m_SENodeList;  //scan eligible list as part of label correcting algorithm 
	
	float* m_node_label_cost;  // label cost 
	int* m_node_predecessor;  // predecessor for nodes
	int* m_node_status_array; // update status 

	int* m_link_predecessor;  // predecessor for this node points to the previous link that updates its label cost (as part of optimality condition) (for easy referencing)

	FILE* pFileAgentPathLog;  // file output

	float* m_link_volume_array; // link volume for all agents assigned in this network (thread)
	float* m_link_cost_array; // link cost 


	std::vector<int>  m_agent_vector; // assigned agents for computing 
	std::vector<int>  m_node_vector; // assigned nodes for computing 
	std::vector<CNode2NodeAccessibility>  m_node2node_accessibility_vector;
	
	
	NetworkForSP()
	{
		pFileAgentPathLog = NULL;
	
	}

	void AllocateMemory(int number_of_nodes, int number_of_links)
	{


		m_node_predecessor = new int[number_of_nodes];
		m_node_status_array = new int[number_of_nodes];
		m_node_label_cost = new float[number_of_nodes];
		m_link_predecessor = new int[number_of_nodes];   // note that, the size is still the number of nodes, as each node has only one link predecessor

	

		m_link_volume_array = new float[number_of_links];

		m_link_cost_array = new float[number_of_links];

		for (int l = 0; l < number_of_links; l++)
		{
			m_link_volume_array[l] = 0.0;
			m_link_cost_array[l] = 1.0; //default value
		}

		//char buffer[256];
		//sprintf_s(buffer, "%s_%d.csv", "agent_path", m_threadNo);

		//pFileAgentPathLog = fopen(buffer, "w");

		//if (pFileAgentPathLog == NULL)
		//{
		//	cout << "Cannot open File" << buffer << endl;
		//	//g_ProgramStop();
		//}
		//else
		//{

		//fprintf(pFileAgentPathLog, "agent_id,origin_node_id,destination_node_id,cost,path_sequence\n");

		//}

	}
	
	~NetworkForSP()
	{

	if (m_node_label_cost != NULL)
		delete m_node_label_cost;

	if (m_node_predecessor != NULL)
		delete m_node_predecessor;

	if (m_node_status_array != NULL)
		delete m_node_status_array;

	if (m_link_predecessor != NULL)
		delete m_link_predecessor;

	if (m_link_volume_array != NULL)
		delete m_link_volume_array;

	if (m_link_cost_array != NULL)
		delete m_link_cost_array;
	
	if(pFileAgentPathLog != NULL)
		fclose(pFileAgentPathLog);
	}

	// SEList: scan eligible List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void SEList_clear()
	{
		m_SENodeList.clear();
	}

	void SEList_push_front(int node)
	{
		m_SENodeList.push_front(node);
	}

	void SEList_push_back(int node)
	{
		m_SENodeList.push_back(node);
	}

	bool SEList_empty()
	{
		return m_SENodeList.empty();
	}

	int SEList_front()
	{
		return m_SENodeList.front();
	}

	void SEList_pop_front()
	{
		m_SENodeList.pop_front();
	}

	int optimal_label_correcting(int origin_node, int destination_node, int departure_time)
		// time-dependent label correcting algorithm with double queue implementation
	{
		int internal_debug_flag = 0;

		if (g_node_vector[origin_node].m_outgoing_node_vector.size() == 0)
		{
			return 0;
		}

		for (int i = 0; i < g_number_of_nodes; i++) //Initialization for all nodes
		{
			m_node_status_array[i] = 0;  // not scanned
			m_node_label_cost[i] = _MAX_LABEL_COST;
			m_node_predecessor[i] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
			m_link_predecessor[i] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
		}

		//Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the delay at origin node

		m_node_label_cost[origin_node] = departure_time;


		SEList_clear();
		SEList_push_back(origin_node);

		while (!SEList_empty())
		{
			int from_node = SEList_front();//pop a node FromID for scanning

			SEList_pop_front();  // remove current node FromID from the SE list

			if (g_shortest_path_debugging_flag)
				fprintf(g_pFileDebugLog, "SP: SE node: %d\n", g_node_vector[from_node].node_id);

			//scan all outbound nodes of the current node
			for (int i = 0; i < g_node_vector[from_node].m_outgoing_node_vector.size(); i++)  // for each link (i,j) belong A(i)
			{
				int to_node = g_node_vector[from_node].m_outgoing_node_vector[i].to_node_seq_no;

				ASSERT(to_node <= g_number_of_nodes);
				bool  b_node_updated = false;

				float new_to_node_cost = m_node_label_cost[from_node] + m_link_cost_array[g_node_vector[from_node].m_outgoing_node_vector[i].link_seq_no];

				if (g_shortest_path_debugging_flag)
				{
					fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d  cost = %d\n",
						g_node_vector[from_node].node_id, 
						g_node_vector[to_node].node_id,
						new_to_node_cost, g_node_vector[from_node].m_outgoing_node_vector[i].cost);
				}



				if (new_to_node_cost < m_node_label_cost[to_node]) // we only compare cost at the downstream node ToID at the new arrival time t
				{

					if (g_shortest_path_debugging_flag)
					{
						fprintf(g_pFileDebugLog, "SP: updating node: %d current cost: %.2f, new cost %.2f\n",
							g_node_vector[to_node].node_id, 
							m_node_label_cost[to_node], new_to_node_cost);
					}

					// update cost label and node/time predecessor

					m_node_label_cost[to_node] = new_to_node_cost;
					m_node_predecessor[to_node] = from_node;  // pointer to previous physical NODE INDEX from the current label at current node and time
					m_link_predecessor[to_node] = g_node_vector[from_node].m_outgoing_node_vector[i].link_seq_no;  // pointer to previous physical NODE INDEX from the current label at current node and time

					b_node_updated = true;

						if (g_shortest_path_debugging_flag)
							fprintf(g_pFileDebugLog, "SP: add node %d into SE List\n",
								g_node_vector[to_node].node_id);

						SEList_push_back(to_node);
						m_node_status_array[to_node] = 1;
				}

			}
		}

		if (destination_node >= 0 && m_node_label_cost[destination_node] < _MAX_LABEL_COST)
			return 1;
		else if (destination_node == -1)
			return 1;  // one to all shortest pat
		else 
			return -1;


	}

	void optimal_label_correcting_for_all_nodes_assigned_to_processor()
	{

		for (int i = 0; i < m_node_vector.size(); i++) //Initialization for all nodes
		{  
			int origin = m_node_vector[i];
			int return_value = optimal_label_correcting(origin, -1, 0);

			if (return_value >= 1) // success 
			{
				for (int j = 0; j < g_number_of_nodes; j++)
				{
					if (j != origin && g_node_vector[j].zone_id >= 1 && m_node_label_cost[j] < _MAX_LABEL_COST)
					{
						CNode2NodeAccessibility element;
						element.from_node_no = origin;
						element.to_node_no = j;
						element.travel_cost = m_node_label_cost[j];

						m_node2node_accessibility_vector.push_back(element);

					}

				}
			}

		}

	}


	void find_path_for_agents_assigned_for_this_thread(int number_of_threads, int assignment_iteration_no)
	{
		for (int l = 0; l < g_number_of_links; l++)
			m_link_volume_array[l] = 0;
			
		// step 1: find shortest path if needed 
		for (int i = 0; i < m_agent_vector.size(); i++)
		{

			CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);
			//step 2 buil SP tree

				// ratio_for_updating_path_at_this_iteration = 1 / (assignment_iteration_no + 1);  //1/1, 1/2, 1/3, 1/4

				int residual = i % (assignment_iteration_no + 1); 

				if (residual != 0)  // no need to compute a new path at this iteration
				{
					continue; // that is, it will reuse the path from the previous iteration, stored at p_agent->path_link_seq_no_vector.
				}
				else
				{
					// move to the next line for finding the shortest path 
				}

			p_agent->path_link_seq_no_vector.clear();  // reset;
			p_agent->path_node_seq_no_vector.clear();  // reset;

			if(p_agent->agent_id%100==0)
			cout << "Finding SP for agent" << p_agent->agent_id << endl;

			int return_value = optimal_label_correcting(g_internal_node_seq_no_map [ p_agent->origin_node_id], g_internal_node_seq_no_map[p_agent->destination_node_id], p_agent->departure_time_in_min);
			//step 3 find the destination node


			if (return_value == -1)
			{
				fprintf(g_pFileDebugLog, "agent %d with can not find destination node,\n", i);
				continue;
			}

			int current_node_seq_no;
			int current_link_seq_no;

			current_node_seq_no = g_internal_node_seq_no_map[p_agent->destination_node_id];
			p_agent->path_cost = m_node_label_cost [ g_internal_node_seq_no_map[p_agent->destination_node_id]];

			if (pFileAgentPathLog != NULL)
			fprintf(pFileAgentPathLog, "%d,%d,%d,%f,", p_agent->agent_id, p_agent->origin_node_id, p_agent->destination_node_id, m_node_label_cost[current_node_seq_no]);
			
			while (current_node_seq_no>=0)
			{
				if (current_node_seq_no >= 0)  // this is valid node 
				{
					current_link_seq_no = m_link_predecessor[current_node_seq_no];

					if(current_link_seq_no>=0)
					{
					p_agent->path_link_seq_no_vector.push_back(current_link_seq_no);
					}

					if (pFileAgentPathLog != NULL)
						fprintf(pFileAgentPathLog, "%d;", g_node_vector[current_node_seq_no].node_id);
				
					p_agent->path_node_seq_no_vector.push_back(current_node_seq_no);

				}


				current_node_seq_no = m_node_predecessor[current_node_seq_no];


			}

			
			if (pFileAgentPathLog != NULL)
				fprintf(pFileAgentPathLog, "\n");


			}

		// step 2: 	scan the shortest path to compute the link volume, 
		for (int i = 0; i < m_agent_vector.size(); i++)
		{

		CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

		for (int l = 0; l < p_agent->path_link_seq_no_vector.size(); l++)  // for each link in the path of this agent
		{
			int link_seq_no = p_agent->path_link_seq_no_vector[l];
			m_link_volume_array[link_seq_no] += p_agent->PCE_factor;
		}

		}

	}

};


int g_number_of_CPU_threads()
{
	int number_of_threads = omp_get_max_threads();

	int max_number_of_threads = 8;

	if (number_of_threads > max_number_of_threads)
		number_of_threads = max_number_of_threads;

	return number_of_threads;

}

NetworkForSP* pNetworkForSP = NULL;
void g_TrafficSimulation()
{
	// given p_agent->path_link_seq_no_vector path link sequence no for each agent

	for (int l = 0; l < g_number_of_links; l++)
	{
		g_link_vector[l].AllocateMemory();
	}
	// simulation
	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		CAgent* p_agent = &(g_agent_vector[a]);
		p_agent->departure_time_in_min = p_agent->departure_time_in_min - g_Simulation_StartTimeInMin;
		p_agent->AllocateMemory();
	}


	for (int t = 0; t < g_number_of_simulation_intervals; t++)
	{
		if(t%10==0)
		cout << "simulation time = " << t/10 << "min" << endl;

		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			if (p_agent->departure_time_in_simulation_interval >= t)
				p_agent->m_bGenereated = true;
		}
//#pragma omp parallel for 

		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);
			
			if (p_agent->m_bGenereated == true &&
				p_agent->m_bCompleteTrip==false && p_agent->m_current_link_seq_no < p_agent->path_link_seq_no_vector.size())
			{
				if (p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no] == t)  // ready to move to the next link
				{
					// check if the current link has sufficient capacity 
					int link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no];

					if (g_link_vector[link_seq_no].m_LinkOutFlowCapacity[t] >= 1)
					{
						if (p_agent->m_current_link_seq_no == p_agent->path_link_seq_no_vector.size() - 1)
						{// end of path

							p_agent->m_bCompleteTrip = true;
							g_link_vector[link_seq_no].m_LinkCumulativeDeparture[t] += 1;

						}else
						{ // not complete the trip

						int next_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no+1];

						p_agent->m_Veh_LinkArrivalTime[p_agent->m_current_link_seq_no + 1] = t;

						p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no + 1] = t + g_link_vector[next_link_seq_no].free_flow_travel_time_in_simu_interval;

						g_link_vector[link_seq_no].m_LinkCumulativeDeparture[t] += 1;
						g_link_vector[next_link_seq_no].m_LinkCumulativeArrival[t] += 1;

						}
						
						//move
						p_agent->m_current_link_seq_no += 1;
						g_link_vector[link_seq_no].m_LinkOutFlowCapacity[t] -= 1;


					}
					else
					{
						p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no] = t + 1;
					}

				}
			}
		}


	}

}

void g_TrafficAssignmentSimulationProcess()
{

	// definte timestamps
	clock_t start_t, end_t, total_t;
	int i;

	start_t = clock();
	int number_of_threads = max(1, g_number_of_CPU_threads());

	number_of_threads = g_number_of_threads;  // step 2: obtain number of  threads for computing 
	pNetworkForSP = new NetworkForSP[number_of_threads]; // create n copies of network, each for a subset of agents to use 


	for (int i = 0; i < number_of_threads; i++)
	{
		pNetworkForSP[i].m_threadNo = i;   // each thread/network has its own thread number. // each network has its own label cost vector for shortest path calculation
		pNetworkForSP[i].AllocateMemory(g_number_of_nodes, g_number_of_links);

	}

	for (int a = 0; a < g_agent_vector.size(); a++)  //assign all agents to the corresponding thread
	{
		int thread_no = a%number_of_threads;  //devide the agent number by total number of threads 
		pNetworkForSP[thread_no].m_agent_vector.push_back(a);
	}


	for (int assignment_iteration_no = 0; assignment_iteration_no < 1; assignment_iteration_no++)
	{
		cout << "Assignment Interval = %d" << assignment_iteration_no << endl;

		for (int l = 0; l < g_number_of_links; l++)
		{
			g_link_vector[l].CalculateBRPFunction();  //based on the g_link_vector[l].flow_volume
		}

#pragma omp parallel for  // step 3: C++ open mp automatically create n threads., each thread has its own computing thread on a cpu core 
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{
			cout << "Processor " << ProcessID << "is calculating shortest paths" << endl;

			//send the link cost to each processor's link cost
			for (int l = 0; l < g_number_of_links; l++)
			{
				pNetworkForSP[ProcessID].m_link_cost_array[l] = g_link_vector[l].cost;
			}

			pNetworkForSP[ProcessID].find_path_for_agents_assigned_for_this_thread(number_of_threads, assignment_iteration_no);
		}

		// tally link volume 
		for (int l = 0; l < g_number_of_links; l++)
		{
			g_link_vector[l].flow_volume = 0.0;
		}

		// tally total volume
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{
			for (int l = 0; l < g_number_of_links; l++)
			{


				g_link_vector[l].flow_volume += pNetworkForSP[ProcessID].m_link_volume_array[l];  // aggregate volume from all threads

				fprintf(g_pFileDebugLog, "\iteration = %d,processor = %d, link no= %d, volume = %f, total volume = %f\n",
					assignment_iteration_no, ProcessID, l, pNetworkForSP[ProcessID].m_link_volume_array[l], g_link_vector[l].flow_volume);

			}
		}



	}

	g_TrafficSimulation();


	FILE* g_pFileLinkMOE = NULL;
	g_pFileLinkMOE = fopen("output_LinkMOE.csv", "w");
	if (g_pFileLinkMOE == NULL)
	{
		cout << "File output_LinkMOE.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	else
	{
		fprintf(g_pFileLinkMOE, "from_node_id,to_node_id,total_link_volume,volume_over_capacity_ratio,speed,travel_time\n");

		for (int l = 0; l < g_link_vector.size(); l++) //Initialization for all nodes
		{
			CLink* pLink = &(g_link_vector[l]);

			fprintf(g_pFileLinkMOE, "%d,%d,%.3f,%.3f,%.3f,%.3f\n",
				g_node_vector[pLink->from_node_seq_no].node_id,
				g_node_vector[pLink->to_node_seq_no].node_id,
				pLink->flow_volume,
				pLink->get_VOC_ratio(),
				pLink->get_speed(),
				pLink->travel_time);
		}


		fclose(g_pFileLinkMOE);
	}

	FILE* g_pFileAgent = NULL;
	g_pFileAgent = fopen("output_agent.csv", "w");
	if (g_pFileAgent == NULL)
	{
		cout << "File output_agent.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	else
	{

		fprintf(g_pFileAgent, "agent_id,origin_node_id,destination_node_id,cost,path_node_sequence,path_time_sequence\n");

		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			std::reverse(std::begin(p_agent->path_node_seq_no_vector),
				std::end(p_agent->path_node_seq_no_vector));
			fprintf(g_pFileAgent, "%d,%d,%d,%f,",
				p_agent->agent_id,
				p_agent->origin_node_id,
				p_agent->destination_node_id,
				p_agent->path_cost
				);

			for (int i = 0; i < p_agent->path_node_seq_no_vector.size(); i++)
			{
				fprintf(g_pFileAgent, "%d;", g_node_vector[p_agent->path_node_seq_no_vector[i]].node_id);
			}

			fprintf(g_pFileAgent, ",");

			for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
			{
				fprintf(g_pFileAgent, "%d:%d;", p_agent->m_Veh_LinkArrivalTime[i], p_agent->m_Veh_LinkDepartureTime[i]);
			}
			fprintf(g_pFileAgent, "\n");

		}
		//
		fclose(g_pFileAgent);
	}


	end_t = clock();

	total_t = (end_t - start_t);

	cout << "CPU Running Time = " << total_t << " milliseconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %ld milliseconds\n", total_t);
	fprintf(g_pFileOutputLog, "CPU Running Time =,%ld, milliseconds\n", total_t);

	fclose(g_pFileOutputLog);
	fclose(g_pFileDebugLog);
}

void  g_AgentBasedODMEProcess(int current_timestamp)
{
	// step 0: set  link capacity to infinity for time before  current_timestamp

	// for ODME iteration i
//	{
   
   // step 1: vehicle generation according to OD loading multiplier alpha(O,D)
	//if (random_number of each possible agent <  loading multiplier(O, D)e.g. 2 / 3)
	//	flag_to_be_generated = 1
	//else
	//	flag_to_be_generated = 0

	// step 2: call 	g_TrafficSimulation(); 
	// based on given input_sensor speed, simulate vehicles (with flag_to_be_generated == 1) along their physical paths, propagate space time trajectories : output: simulated link flow and density (travel time with waiting time subject to capacity constraints)
	// capacity constraints are enforced after time current timestamp, for different future scenarios, e.g. different work zones, ramp metering or signal control methods 

   // step 3: calculate link measurement deviation for flow volume and density 

	//step 4: update OD loading multiplier alpha(O,D)
   //scan each simulated agent, cumulate path marginal, update the OD loading multiplier alpha(O,D) according to the following formula
	//Multiplier(O, D) += Step_size *  total path marginal(e.g.observed - simulated) :
	//	E.g.observed higher,  simulated needs to be more,
	//	marginal is positive - > multiplier increases->more chance to be generated in the next iteration
	//

	for (int ODME_iteration = 0; ODME_iteration < 20; ODME_iteration++)
	{
		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			// step 1: vehicle generation according to OD loading multiplier alpha(O,D)
			//if (random_number of each possible agent <  loading multiplier(O, D)e.g. 2 / 3)
			//	flag_to_be_generated = 1
			//else
			//	flag_to_be_generated = 0

			CAgent* p_agent = &(g_agent_vector[a]);
			p_agent->VehicleGeneration();
		}

		// step 2: call 	g_TrafficSimulation(); 
		// based on given input_sensor speed, simulate vehicles (with flag_to_be_generated == 1) along their physical paths, propagate space time trajectories : output: simulated link flow and density (travel time with waiting time subject to capacity constraints)
		// capacity constraints are enforced after time current timestamp, for different future scenarios, e.g. different work zones, ramp metering or signal control methods 

		g_TrafficSimulation();

		// step 3: calculate link measurement deviation for flow volume and density 

		for (int l = 0; l < g_number_of_links; l++)
		{
			g_link_vector[l].UpdateMeasurementDeviation();
		}

		// step 3: calculate link measurement deviation for flow volume and density 


		//step 4: update OD loading multiplier alpha(O,D)
		//scan each simulated agent, cumulate path marginal, update the OD loading multiplier alpha(O,D) according to the following formula
		//Multiplier(O, D) += Step_size *  total path marginal(e.g.observed - simulated) :
		//	E.g.observed higher,  simulated needs to be more,
		//	marginal is positive - > multiplier increases->more chance to be generated in the next iteration
		//
	}

}
int main(int argc, TCHAR* argv[], TCHAR* envp[])
{
	g_pFileDebugLog = fopen("Debug.txt", "w");
	if (g_pFileDebugLog == NULL)
	{ 
		cout << "File Debug.txt cannot be opened." << endl;
		g_ProgramStop();
	}
	 g_pFileOutputLog = fopen("output_solution.csv", "w");
	 if (g_pFileOutputLog == NULL)
	 {
		 cout << "File output_solution.csv cannot be opened." << endl;
		 g_ProgramStop();
	 }


	g_ReadInputData();  // step 1: read input data of network and demand agent 

	for (int o = 0; o < g_number_of_zones; o++)
		for (int d = 0; d < g_number_of_zones; d++)
		{
			g_OD_loading_multiplier[o][d] = g_loading_multiplier;
		}



	// mode 1: void g_TrafficAssignmentSimulationProcess()

	g_TrafficAssignmentSimulationProcess();
	// mode 2: g_AgentBasedODMEProcess()

	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;
	cout << "done." << endl;
	delete[] pNetworkForSP;

	g_node_vector.clear();
	g_link_vector.clear();

	return 1;
}
