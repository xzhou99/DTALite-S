//  Portions Copyright 2010 Xuesong Zhou

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of DTALite.

//    DTALite is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    DTALite is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with DTALite.  If not, see <http://www.gnu.org/licenses/>.

// DTALite.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <list> 
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "CSVParser.h"
#include "DTALite-S.h"
#include <functional>
#include<stdio.h>   
#include<tchar.h>

#define _MAX_LABEL_COST 99999
#define _MAX_NUMBER_OF_PAX 100
#define _MAX_NUMBER_OF_VEHICLES 100
#define _MAX_NUMBER_OF_TIME_INTERVALS 150
#define _MAX_NUMBER_OF_DEMAND_TYPES 20
#define _MAX_NUMBER_OF_PHYSICAL_NODES 400000

#define _MAX_STATES 2
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
FILE* g_pFileDebugLog_LR = NULL;
FILE* g_pFileDebugLog_ADMM = NULL;
FILE* g_pTSViewOutput = NULL;
FILE* g_pNGSIMOuputLog = NULL;
FILE* g_ptrainDelayLog = NULL;

int enum_waiting_link_type = 5;  //To do: to be changed. 
int enum_road_capacity_link_type = 0;  //To do: to be changed. 
int enum_request_link_type = 100;  //To do: to be changed. 

int g_number_of_threads = 4;
int g_shortest_path_debugging_flag = 0;
int g_number_of_agents;
int g_number_of_demand_types = 1;

double g_number_of_seconds_per_interval = 6;  // 0.2 seconds for 300 intervals per min
int g_number_of_simulation_intervals = 60 * 60 / g_number_of_seconds_per_interval;    // 40min
int g_number_of_optimization_time_intervals = 60;

int g_Simulation_StartTimeInMin = 9999;
int g_Simulation_EndTimeInMin = 0;
int g_start_simu_interval_no, g_end_simu_interval_no;

int g_Post_Simulation_DurationInMin = 120;

int g_dp_algorithm_debug_flag = 0;
float g_penalty_RHO = 20;
int g_optimization_method = 2;
//g_convert_abs_simu_interval_to_relative_simu_interval
int g_A2R_simu_interval(int abs_simu_interval)
{
	return abs_simu_interval - g_start_simu_interval_no;

}

// 6 seconds per interval
// 3600 -> 6
// 1800 -> 3
// 900 -> 1.5

std::map<int, int> g_link_key_to_seq_no_map;  // hush table, map key to internal link sequence no. 


//mfd
int g_TAU;


std::map<int, int> g_internal_node_seq_no_map;  // hush table, map external node number to internal node sequence no. 
std::map<int, int> g_internal_node_seq_no_to_node_id_map;  // hush table, map external node number to internal node sequence no. 


long g_GetLinkSeqNo(int from_node_id, int to_node_id)
{
	if (g_internal_node_seq_no_map.find(from_node_id) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	if (g_internal_node_seq_no_map.find(from_node_id) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	int from_node_seq_no = g_internal_node_seq_no_map[from_node_id];
	int to_node_seq_no = g_internal_node_seq_no_map[to_node_id];

	long link_key = from_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + to_node_seq_no;

	if (g_link_key_to_seq_no_map.find(link_key) != g_link_key_to_seq_no_map.end())
		return g_link_key_to_seq_no_map[link_key];
	else
		return -1;
}


class CLink
{
public:
	CLink()  // construction 
	{
		m_End_of_RedTime_in_simu_interval = 0;
		external_link_id = 0;
		service_type = 0;
		service_price = 0;
		VRP_load_id = -1;
		base_price = 10;
		VRP_load_difference = 0;

		cost = 0;
		BRP_alpha = 0.15f;
		BRP_beta = 4.0f;
		link_capacity = 1000;
		free_flow_travel_time_in_min = 1;
		flow_volume = 0;
		number_of_lanes = 1;
		// mfd
		mfd_zone_id = 0;

		m_LinkOutFlowCapacity = NULL;
		m_LinkInFlowCapacity = NULL;
		m_LinkCumulativeArrival = NULL;
		m_LinkCumulativeDeparture = NULL;
		m_LinkCumulativeVirtualDelay = NULL;

		max_allowed_waiting_time = 0;

		VRP_time_window_begin = -1;
		VRP_time_window_end = 10000;
	}

	~CLink()
	{
		DeallocateMemory();
	}

	std::list<int>  m_waiting_traveler_queue;

	// all alocated as relative time
	float* m_LinkOutFlowCapacity;
	float* m_LinkInFlowCapacity;
	int m_End_of_RedTime_in_simu_interval;

	int* m_LinkCumulativeArrival;
	int* m_LinkCumulativeDeparture;
	int* m_LinkCumulativeVirtualDelay;
	float* m_LinkTravelTime;

	int m_CumulativeArrivalCount;
	int m_CumulativeDepartureCount;
	int m_CumulativeVirtualDelayCount;



	int VRP_time_window_begin, VRP_time_window_end;
	float base_price;
	std::vector<int> travel_time_vector;
	std::vector<float> time_dependent_LR_multiplier_vector, time_dependent_external_cost_vector, time_dependent_ADMM_multiplier_vector;

	std::vector<int> time_dependent_visit_counts, time_dependent_ADMM_visit_counts,
		time_depedent_capacity_vector;

	//ADMM_multiplier_matrix is used in the searching process and updated by LR_multiplier_matrix
	int max_allowed_waiting_time;

	void Setup_State_Dependent_Data_Matrix(int number_of_optimization_time_intervals)
	{

		for (int t = 0; t < number_of_optimization_time_intervals; t++)
		{
			time_dependent_visit_counts.push_back(0);
			time_dependent_ADMM_visit_counts.push_back(0);
			time_dependent_LR_multiplier_vector.push_back(0);
			time_depedent_capacity_vector.push_back(link_capacity);

			time_dependent_external_cost_vector.push_back(0);
			time_dependent_ADMM_multiplier_vector.push_back(0);
			travel_time_vector.push_back((int)(free_flow_travel_time_in_min));  //assume simulation time interval as free-flow travel time per cell 

		}

		VRP_time_window_begin = max(0, VRP_time_window_begin);
		VRP_time_window_end = min(number_of_optimization_time_intervals - 1, VRP_time_window_end);

	}

	float GetCapacityPerSimuInterval(float link_capacity_per_hour)
	{
		return link_capacity_per_hour / 3600.0 *g_number_of_seconds_per_interval;
	}

	void AllocateMemory()
	{
		m_LinkOutFlowCapacity = new float[g_number_of_simulation_intervals];
		m_LinkInFlowCapacity = new float[g_number_of_simulation_intervals];
		m_LinkCumulativeArrival = new int[g_number_of_simulation_intervals];
		m_LinkCumulativeDeparture = new int[g_number_of_simulation_intervals];
		m_LinkCumulativeVirtualDelay = new int[g_number_of_simulation_intervals];
		m_LinkTravelTime = new float[g_number_of_simulation_intervals];


		for (int t = 0; t < g_number_of_simulation_intervals; t++)
		{
			m_LinkOutFlowCapacity[t] = GetCapacityPerSimuInterval(link_capacity);
			m_LinkCumulativeArrival[t] = 0;
			m_LinkCumulativeDeparture[t] = 0;
			m_LinkCumulativeVirtualDelay[t] = 0;

		}

		free_flow_travel_time_in_simu_interval = int(free_flow_travel_time_in_min*60.0 / g_number_of_seconds_per_interval + 0.5);
	}

	void ResetMOE()
	{
		m_CumulativeArrivalCount = 0;
		m_CumulativeDepartureCount = 0;
		m_CumulativeVirtualDelayCount = 0;


	}

	void DeallocateMemory()
	{
		//if(m_LinkOutFlowCapacity != NULL) delete m_LinkOutFlowCapacity;
		//if (m_LinkInFlowCapacity != NULL) delete m_LinkInFlowCapacity;
		//if (m_LinkCumulativeArrival != NULL) delete m_LinkCumulativeArrival;
		//if (m_LinkCumulativeDeparture != NULL) delete m_LinkCumulativeDeparture;
		//if (m_LinkTravelTime != NULL) delete m_LinkTravelTime;

	}

	int external_link_id;
	int link_seq_no;  // internal seq no
	int from_node_seq_no;
	int to_node_seq_no;
	float cost;
	float free_flow_travel_time_in_min;
	int free_flow_travel_time_in_simu_interval;
	int number_of_lanes;
	bool demand_type_code[_MAX_NUMBER_OF_DEMAND_TYPES];
	float demand_type_TTcost[_MAX_NUMBER_OF_DEMAND_TYPES];

	int type;
	int service_type; // 0: moving, -1: drop off, +1, pick up

	float service_price; // for pick up or drop off
	int VRP_load_id;
	int VRP_group_id;

	int VRP_load_difference; // we use a single point time window now

	float link_capacity;
	float flow_volume;
	float travel_time;
	float BRP_alpha;
	float BRP_beta;
	float length;
	// mfd
	int mfd_zone_id;

	void CalculateBPRFunctionAndCost()
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

	float get_link_in_flow_per_min(int time_in_min)
	{
		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin - 1)
		{
			int next_time_in_internval = (time_in_min + 1) * 60 / g_number_of_seconds_per_interval;
			int time_in_interval = time_in_min * 60 / g_number_of_seconds_per_interval;
			return m_LinkCumulativeArrival[g_A2R_simu_interval(next_time_in_internval)] - m_LinkCumulativeArrival[g_A2R_simu_interval(time_in_interval)];
		}
		else
			return 0;

	}

	float get_link_out_flow_per_min(int time_in_min)
	{
		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin - 1)
		{
			int next_time_in_internval = (time_in_min + 1) * 60 / g_number_of_seconds_per_interval;
			int time_in_interval = time_in_min * 60 / g_number_of_seconds_per_interval;

			return m_LinkCumulativeDeparture[g_A2R_simu_interval(next_time_in_internval)] - m_LinkCumulativeDeparture[g_A2R_simu_interval(time_in_interval)];
		}
		else
			return 0;

	}

	float get_number_of_vehicles(int time_in_min)
	{
		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin)
		{
			int time_in_interval = g_A2R_simu_interval((time_in_min) * 60 / g_number_of_seconds_per_interval);


			return m_LinkCumulativeArrival[time_in_interval] - m_LinkCumulativeDeparture[time_in_interval];
		}
		else
			return 0;

	}


	float get_avg_delay_in_min(int time_in_min, int time_duration)
	{

		if (m_LinkCumulativeVirtualDelay != NULL && time_in_min + time_duration < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin)
		{
			int time_next_in_interval = g_A2R_simu_interval((time_in_min + time_duration) * 60 / g_number_of_seconds_per_interval);
			int time_in_interval = g_A2R_simu_interval(time_in_min * 60 / g_number_of_seconds_per_interval);
			float total_delay = (m_LinkCumulativeVirtualDelay[time_next_in_interval] - m_LinkCumulativeVirtualDelay[time_in_interval]);
			return total_delay / max(1, get_number_of_vehicles(time_in_min));
		}
		else
		{
			return 0;
		}
	}

};


class CNode
{
public:
	CNode()
	{
		zone_id = 0;
		accessible_node_count = 0;
		bOriginNode_ForAgents = false;
		m_OriginNodeSeqNo = -1;
	}

	int accessible_node_count;

	int node_seq_no;  // sequence number 
	int external_node_id;      //external node number 
	int zone_id;
	double x;
	double y;

	bool bOriginNode_ForAgents;
	int m_OriginNodeSeqNo;

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
		agent_vector_seq_no = -1;
		agent_service_type = 0;  //0: pax vehicle 1: travler 2: scheduled transportation vehicle
		m_bMoveable = 1;
		fixed_path_flag = 0;
		vehicle_seat_capacity = 1;
		m_bGenereated = false;
		PCE_factor = 1.0;
		path_cost = 0;
		m_Veh_LinkArrivalTime_in_simu_interval = NULL;
		m_Veh_LinkDepartureTime_in_simu_interval = NULL;
		m_bCompleteTrip = false;
		departure_time_in_min = 0;
		// vrp

		transportation_time_cost = 0;
		schedule_early_cost = 0;
		schedule_delay_cost = 0;

		earliest_departure_time = 0;
		departure_time_window = 1;

		latest_arrival_time = 0;
		arrival_time_window = 1;

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



	int fixed_path_flag;
	int demand_type;
	int agent_id;
	int agent_vector_seq_no;
	int agent_service_type;
	int m_bMoveable;
	int origin_node_id;
	int destination_node_id;

	int origin_zone_seq_no;
	int destination_zone_seq_no;

	float departure_time_in_min;
	int departure_time_in_simu_interval;

	float PCE_factor;  // passenger car equivalent : bus = 3
	float path_cost;
	std::vector<int> path_link_seq_no_vector;
	int m_path_link_seq_no_vector_size;

	std::vector<int> path_node_id_vector;
	std::vector<int> path_schedule_time_vector;

	int m_current_link_seq_no;
	int* m_Veh_LinkArrivalTime_in_simu_interval;
	int* m_Veh_LinkDepartureTime_in_simu_interval;

	int vehicle_seat_capacity;
	int VRP_group_id;

	std::list<int>  m_PassengerList;

	vector<int> set_of_allowed_links;
	vector<int> m_set_of_allowed_links_flag;
	vector<int> set_of_allowed_nodes;
	vector<int> set_of_allowed_links_LR;
	vector<int> m_set_of_allowed_links_flag_LR;

	// STS
	float transportation_time_cost;
	float schedule_early_cost;
	float schedule_delay_cost;

	float earliest_departure_time;
	int departure_time_in_simulation_interval;
	float departure_time_window;

	float latest_arrival_time;
	float arrival_time_window;
	std::vector<int> path_timestamp_vector;

	// STS
	void Pickup(int p)
	{
		if (m_PassengerList.size() < vehicle_seat_capacity)
		{
			m_PassengerList.push_back(p);
		}

	}


	int GetRemainingCapacity()
	{

		return vehicle_seat_capacity - m_PassengerList.size();
	}

	//above are simulated 

	bool m_bCompleteTrip;
	bool operator<(const CAgent &other) const
	{
		return departure_time_in_min < other.departure_time_in_min;
	}

	void AllocateMemory()
	{
		if (m_Veh_LinkArrivalTime_in_simu_interval == NULL)
		{
			m_current_link_seq_no = 0;
			m_Veh_LinkArrivalTime_in_simu_interval = new int[path_link_seq_no_vector.size()];
			m_Veh_LinkDepartureTime_in_simu_interval = new int[path_link_seq_no_vector.size()];

			for (int i = 0; i < path_link_seq_no_vector.size(); i++)
			{
				m_Veh_LinkArrivalTime_in_simu_interval[i] = -1;
				m_Veh_LinkDepartureTime_in_simu_interval[i] = -1;

			}


			m_path_link_seq_no_vector_size = path_link_seq_no_vector.size();
			departure_time_in_simu_interval = int(departure_time_in_min*60.0 / g_number_of_seconds_per_interval + 0.5);  // round off

			if (path_link_seq_no_vector.size() > 0)
			{
				m_Veh_LinkArrivalTime_in_simu_interval[0] = departure_time_in_simu_interval;

				int FirstLink = path_link_seq_no_vector[0];

				m_Veh_LinkDepartureTime_in_simu_interval[0] = m_Veh_LinkArrivalTime_in_simu_interval[0] + g_link_vector[FirstLink].free_flow_travel_time_in_simu_interval;

				int relative_simulation_interval = g_A2R_simu_interval(departure_time_in_simu_interval);
				g_link_vector[FirstLink].m_LinkCumulativeArrival[relative_simulation_interval] += 1;
			}
		}
	}

	void DeallocateMemory()
	{
		//if (m_Veh_LinkArrivalTime_in_simu_interval != NULL) delete m_Veh_LinkArrivalTime_in_simu_interval;
		//if (m_Veh_LinkDepartureTime_in_simu_interval != NULL) delete m_Veh_LinkDepartureTime_in_simu_interval;

	}
	std::map<int, int> m_VRP_ADMM_link_time_map;

};

class CActiveAgentElement
{
public:
	CActiveAgentElement()
	{
		pAgent = NULL;
		bActive = true;
	}


	CAgent* pAgent;
	bool bActive;
};

vector<CAgent> g_agent_vector;
std::map<int, int> g_map_agent_id_to_agent_vector_seq_no;

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_zones = 0;

int g_AddNewServiceNode(int load_id, int physical_node_id, float x, float y)
{
	int node_id = load_id;
	std::map<int, int> node_id_map;

	int internal_node_seq_no = g_number_of_nodes;

	if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
	{
		return -1; //has been defined
	}
	g_internal_node_seq_no_map[node_id] = internal_node_seq_no;
	g_internal_node_seq_no_to_node_id_map[internal_node_seq_no] = node_id;

	CNode node;  // create a node object 

	node.external_node_id = node_id;
	node.node_seq_no = internal_node_seq_no;
	node.zone_id;

	node.x = x;
	node.y = y;

	g_node_vector.push_back(node);  // push it to the global node vector

	g_number_of_nodes++;
	if (g_number_of_nodes % 1000 == 0)
		cout << "reading " << g_number_of_nodes << " nodes.. " << endl;

	return internal_node_seq_no;
}

void g_AddServiceLinksBasedonPaxData(int physical_origin_node,
	int physical_destination_node,
	int service_type,
	int VRP_load_id,
	int VRP_group_id,
	int VRP_load,
	int	VRP_time_window_begin,
	int VRP_time_window_end)
{
	// step 1: add service node node
	int new_service_node = 100000 * VRP_group_id + VRP_load_id;
	int internal_physical_origin_node = g_internal_node_seq_no_map[physical_origin_node];

	float x = g_node_vector[internal_physical_origin_node].x;
	float y = g_node_vector[internal_physical_origin_node].y;
	int internal_service_node_seq_no = g_AddNewServiceNode(new_service_node, physical_origin_node, x, y);

	CLink link;  // create a link object 

	// first link from physical origin to service node 
	link.from_node_seq_no = internal_physical_origin_node;
	link.to_node_seq_no = internal_service_node_seq_no;

	g_node_vector[internal_physical_origin_node].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link
	long link_key = internal_physical_origin_node * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_service_node_seq_no;
	link.link_seq_no = g_number_of_links;
	g_link_key_to_seq_no_map[link_key] = link.link_seq_no;

	link.type = 1;
	link.service_type = service_type;
	link.VRP_load_id = VRP_load_id;
	link.VRP_group_id = VRP_group_id;
	link.VRP_time_window_begin = VRP_time_window_begin;
	link.VRP_time_window_end = VRP_time_window_end;
	link.VRP_load_difference = VRP_load;
	link.number_of_lanes = 1;
	link.link_capacity = 1;

	for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
	{
		link.demand_type_code[d] = true;
		link.demand_type_TTcost[d] = 0;
	}

	link.free_flow_travel_time_in_min = 1;
	link.length = 1;
	link.cost = 0; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate g_A2R_simu_interval 

	g_link_vector.push_back(link);
	g_number_of_links++;

	// second link from service node back to origin node

	link.from_node_seq_no = internal_service_node_seq_no;
	link.to_node_seq_no = internal_physical_origin_node;

	g_node_vector[internal_service_node_seq_no].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link
	link_key = internal_service_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_physical_origin_node;
	link.link_seq_no = g_number_of_links;
	g_link_key_to_seq_no_map[link_key] = link.link_seq_no;
	link.type = 1;
	link.service_type = 0;
	link.number_of_lanes = 1;
	link.link_capacity = 1;

	for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
	{
		link.demand_type_code[d] = true;
		link.demand_type_TTcost[d] = 0;
	}

	link.free_flow_travel_time_in_min = 1;
	link.length = 1;
	link.cost = 0; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate g_A2R_simu_interval 

	g_link_vector.push_back(link);
	g_number_of_links++;


	if (g_number_of_links % 1000 == 0)
		cout << "creating " << g_number_of_links << " links.. " << endl;

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
			g_internal_node_seq_no_to_node_id_map[internal_node_seq_no] = node_id;

			parser.GetValueByFieldName("x", x, false);
			parser.GetValueByFieldName("y", y, false);


			CNode node;  // create a node object 

			node.external_node_id = node_id;
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
	else
	{
		cout << "input_node.csv is not opened." << endl;
		g_ProgramStop();
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

			parser_link.GetValueByFieldName("link_id", link.external_link_id);

			link.from_node_seq_no = internal_from_node_seq_no;
			link.to_node_seq_no = internal_to_node_seq_no;
			link.link_seq_no = g_number_of_links;
			//link.to_node_seq_no = internal_to_node_seq_no;

			parser_link.GetValueByFieldName("link_type", link.type);
			parser_link.GetValueByFieldName("service_type", link.service_type, false);

			if (link.service_type != 0)
			{
				parser_link.GetValueByFieldName("VRP_load_id", link.VRP_load_id, false);
				parser_link.GetValueByFieldName("VRP_group_id", link.VRP_group_id, false);
				parser_link.GetValueByFieldName("VRP_time_window_begin", link.VRP_time_window_begin, false);
				parser_link.GetValueByFieldName("VRP_time_window_end", link.VRP_time_window_end, false);
				parser_link.GetValueByFieldName("VRP_load_difference", link.VRP_load_difference, false);
			}
			float length = 1; // km or mile
			float speed_limit = 1;
			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("speed_limit", speed_limit);
			parser_link.GetValueByFieldName("base_price", link.base_price);
			parser_link.GetValueByFieldName("BPR_alpha_term", link.BRP_alpha);
			parser_link.GetValueByFieldName("BPR_beta_term", link.BRP_beta);

			int number_of_lanes = 1;
			float lane_cap = 1000;
			parser_link.GetValueByFieldName("number_of_lanes", link.number_of_lanes);
			parser_link.GetValueByFieldName("lane_cap", lane_cap);

			link.link_capacity = lane_cap* number_of_lanes;

			string demand_type_code;

			for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
			{
				link.demand_type_code[d] = true;
				link.demand_type_TTcost[d] = 0;
			}

			parser_link.GetValueByFieldName("demand_type_code", demand_type_code);

			if (demand_type_code.size() > 0)  //demand type code has a string
			{
				for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
				{
					link.demand_type_code[d] = false;
					CString demand_type_number;
					demand_type_number.Format(_T("%d"), d);

					std::string str_number = CString2StdString(demand_type_number);

					if (demand_type_code.find(str_number) != std::string::npos)   // find this number
					{
						link.demand_type_code[d] = true;  // allow this demand type
					}
				}
			}


			for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
			{
				CString demand_type_number;
				demand_type_number.Format(_T("demand_type_%d_TTcost"), d);

				std::string str_number = CString2StdString(demand_type_number);
				parser_link.GetValueByFieldName(str_number, link.demand_type_TTcost[d]);

			}
			link.free_flow_travel_time_in_min = length / speed_limit * 60;

			float external_travel_time = -1;
			parser_link.GetValueByFieldName("external_travel_time", external_travel_time, false);

			if (external_travel_time >= 0.1)
			{  // reset 
				link.free_flow_travel_time_in_min = external_travel_time;
			}

			link.length = length;
			link.cost = length / speed_limit * 60; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate g_A2R_simu_interval 

			g_node_vector[internal_from_node_seq_no].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link

			long link_key = internal_from_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_to_node_seq_no;

			g_link_key_to_seq_no_map[link_key] = link.link_seq_no;
			g_link_vector.push_back(link);
			link.CalculateBPRFunctionAndCost(); // initial link travel time value
			g_number_of_links++;

			if (g_number_of_links % 1000 == 0)
				cout << "reading " << g_number_of_links << " links.. " << endl;
		}
	}
	else
	{
		cout << "input_link.csv is not opened." << endl;
		g_ProgramStop();
	}


	cout << "number of links = " << g_number_of_links << endl;

	fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);

	parser_link.CloseCSVFile();

	g_number_of_agents = 0;
	CCSVParser parser_agent;
	std::vector<int> path_node_sequence;
	string path_node_sequence_str;

	std::vector<int> path_schedule_time_sequence;
	string path_schedule_time_sequence_str;

	if (parser_agent.OpenCSVFile("input_agent.csv", true))   // read agent as demand input 
	{
		while (parser_agent.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			CAgent agent;  // create an agent object 
			if (parser_agent.GetValueByFieldName("agent_id", agent.agent_id) == false)
				continue;

			parser_agent.GetValueByFieldName("agent_service_type", agent.agent_service_type);

			agent.m_RandomSeed = agent.agent_id;

			int origin_node_id = 0;
			int destination_node_id = 0;
			parser_agent.GetValueByFieldName("from_origin_node_id", origin_node_id, false);

			agent.origin_node_id = origin_node_id;
			parser_agent.GetValueByFieldName("to_destination_node_id", destination_node_id, false);
			agent.destination_node_id = destination_node_id;

			if (g_internal_node_seq_no_map.find(origin_node_id) == g_internal_node_seq_no_map.end() || g_internal_node_seq_no_map.find(destination_node_id) == g_internal_node_seq_no_map.end())
				continue;

			parser_agent.GetValueByFieldName("from_zone_id", agent.origin_zone_seq_no, false);
			parser_agent.GetValueByFieldName("to_zone_id", agent.destination_zone_seq_no, false);

			parser_agent.GetValueByFieldName("demand_type", agent.demand_type, false);

			if (agent.demand_type > g_number_of_demand_types)
				g_number_of_demand_types = agent.demand_type;

			ASSERT(g_number_of_demand_types + 1 < _MAX_NUMBER_OF_DEMAND_TYPES);

			parser_agent.GetValueByFieldName("VRP_group_id", agent.VRP_group_id);
			parser_agent.GetValueByFieldName("vehicle_seat_capacity", agent.vehicle_seat_capacity);
			parser_agent.GetValueByFieldName("fixed_path_flag", agent.fixed_path_flag);
			parser_agent.GetValueByFieldName("departure_time_in_min", agent.departure_time_in_min);


			if (agent.departure_time_in_min < g_Simulation_StartTimeInMin)
				g_Simulation_StartTimeInMin = agent.departure_time_in_min;

			if (agent.departure_time_in_min > g_Simulation_EndTimeInMin)
				g_Simulation_EndTimeInMin = agent.departure_time_in_min;

			parser_agent.GetValueByFieldName("PCE", agent.PCE_factor);
			parser_agent.GetValueByFieldName("path_node_sequence", path_node_sequence_str);

			agent.path_node_id_vector = ParseLineToIntegers(path_node_sequence_str);

			parser_agent.GetValueByFieldName("path_schdule_time_sequence", path_schedule_time_sequence_str);

			agent.path_schedule_time_vector = ParseLineToIntegers(path_schedule_time_sequence_str);

			if (agent.path_node_id_vector.size() >= 2)
			{
				for (int n = 0; n < agent.path_node_id_vector.size() - 1; n++)
				{
					int link_seq_no = g_GetLinkSeqNo(agent.path_node_id_vector[n], agent.path_node_id_vector[n + 1]);

					if (link_seq_no == -1)
					{
						// trace: error
						break;
					}

					agent.path_link_seq_no_vector.push_back(link_seq_no);
				}

			}
			//To Do 1: load agent path from field path_node_sequence
			// initial loading multiplier: 0.66666

			if (agent.agent_service_type == 2)
			{
				parser_agent.GetValueByFieldName("earliest_departure_time", agent.earliest_departure_time);
				parser_agent.GetValueByFieldName("departure_time_window", agent.departure_time_window);
				parser_agent.GetValueByFieldName("latest_arrival_time", agent.latest_arrival_time);
				parser_agent.GetValueByFieldName("arrival_time_window", agent.arrival_time_window);
			}

			parser_agent.GetValueByFieldName("transportation_time_cost", agent.transportation_time_cost);
			parser_agent.GetValueByFieldName("schedule_early_cost", agent.schedule_early_cost);
			parser_agent.GetValueByFieldName("schedule_delay_cost", agent.schedule_delay_cost);

			agent.departure_time_in_simulation_interval = agent.earliest_departure_time / g_number_of_seconds_per_interval;  // covert departure time in min to an integer value of simulation time intervals

			if (agent.latest_arrival_time >= g_number_of_optimization_time_intervals)
				g_number_of_optimization_time_intervals = agent.latest_arrival_time + 1;

			g_agent_vector.push_back(agent);
			g_number_of_agents++;
			if (g_number_of_agents % 1000 == 0)
				cout << "reading = " << g_number_of_agents / 1000 << " k agents..." << endl;

		}
	}
	else
	{
		cout << "input_agent.csv is not opened." << endl;
		g_ProgramStop();
	}

	cout << "number of agents = " << g_agent_vector.size() << endl;

	cout << " Sort agents... " << endl;
	std::sort(g_agent_vector.begin(), g_agent_vector.end());

	// simulation
	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		g_agent_vector[a].agent_vector_seq_no = a;

		g_map_agent_id_to_agent_vector_seq_no[g_agent_vector[a].agent_id] = a;// based on agent_id to find agent_vector
	}
	cout << " Sorting ends. ..." << endl;

	// use absoluate time scale
	g_start_simu_interval_no = g_Simulation_StartTimeInMin * 60 / g_number_of_seconds_per_interval;
	g_end_simu_interval_no = g_start_simu_interval_no + g_number_of_simulation_intervals;


	parser_agent.CloseCSVFile();

}

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


	int m_private_origin_seq_no;
	std::vector<int>  m_agent_vector; // assigned agents for computing 
	std::vector<int>  m_node_vector; // assigned nodes for computing 

	NetworkForSP()
	{
		pFileAgentPathLog = NULL;
		m_private_origin_seq_no = -1;

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

		if (pFileAgentPathLog != NULL)
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

	int optimal_label_correcting(int origin_node, int destination_node, int departure_time, int demand_type)
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

		//Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the g_A2R_simu_interval at origin node

		m_node_label_cost[origin_node] = departure_time;


		SEList_clear();
		SEList_push_back(origin_node);

		while (!SEList_empty())
		{
			int from_node = SEList_front();//pop a node FromID for scanning

			SEList_pop_front();  // remove current node FromID from the SE list

			if (g_shortest_path_debugging_flag)
				fprintf(g_pFileDebugLog, "SP: SE node: %d\n", g_node_vector[from_node].external_node_id);

			//scan all outbound nodes of the current node
			for (int i = 0; i < g_node_vector[from_node].m_outgoing_node_vector.size(); i++)  // for each link (i,j) belong A(i)
			{
				int to_node = g_node_vector[from_node].m_outgoing_node_vector[i].to_node_seq_no;

				ASSERT(to_node <= g_number_of_nodes);
				bool  b_node_updated = false;

				int link_seq_no = g_node_vector[from_node].m_outgoing_node_vector[i].link_seq_no;

				if (g_link_vector[link_seq_no].demand_type_code[demand_type] == false)
					continue;

				float new_to_node_cost = m_node_label_cost[from_node] + m_link_cost_array[link_seq_no] + g_link_vector[link_seq_no].demand_type_TTcost[demand_type];

				if (g_shortest_path_debugging_flag)
				{
					fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d  cost = %f\n",
						g_node_vector[from_node].external_node_id,
						g_node_vector[to_node].external_node_id,
						new_to_node_cost/*, g_node_vector[from_node].m_outgoing_node_vector[i].cost*/);
				}

				if (new_to_node_cost < m_node_label_cost[to_node]) // we only compare cost at the downstream node ToID at the new arrival time t
				{

					if (g_shortest_path_debugging_flag)
					{
						fprintf(g_pFileDebugLog, "SP: updating node: %d current cost: %.2f, new cost %.2f\n",
							g_node_vector[to_node].external_node_id,
							m_node_label_cost[to_node], new_to_node_cost);
					}

					// update cost label and node/time predecessor

					m_node_label_cost[to_node] = new_to_node_cost;
					m_node_predecessor[to_node] = from_node;  // pointer to previous physical NODE INDEX from the current label at current node and time
					m_link_predecessor[to_node] = g_node_vector[from_node].m_outgoing_node_vector[i].link_seq_no;  // pointer to previous physical NODE INDEX from the current label at current node and time

					b_node_updated = true;

					if (g_shortest_path_debugging_flag)
						fprintf(g_pFileDebugLog, "SP: add node %d into SE List\n",
							g_node_vector[to_node].external_node_id);

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




	void find_path_for_agents_assigned_for_this_thread(int number_of_threads, int assignment_iteration_no)
	{
		for (int l = 0; l < g_number_of_links; l++)
			m_link_volume_array[l] = 0;

		// perform shortest path calculation

		if (m_private_origin_seq_no < 0)
			return;

		for (int demand_type = 0; demand_type <= g_number_of_demand_types; demand_type++)
		{
			int return_value = optimal_label_correcting(m_private_origin_seq_no, -1, 0, demand_type);


			// step 1: find shortest path if needed 
			for (int i = 0; i < m_agent_vector.size(); i++)
			{

				CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

				if (p_agent->fixed_path_flag == 1)
					continue;

				if (p_agent->demand_type != demand_type)
					continue;
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
				p_agent->path_node_id_vector.clear();  // reset;


				//step 3 find the destination node


				if (return_value == -1)
				{
					fprintf(g_pFileDebugLog, "agent %d with can not find destination node,\n", i);
					continue;
				}

				int current_node_seq_no;
				int current_link_seq_no;

				current_node_seq_no = g_internal_node_seq_no_map[p_agent->destination_node_id];
				p_agent->path_cost = m_node_label_cost[g_internal_node_seq_no_map[p_agent->destination_node_id]];

				if (pFileAgentPathLog != NULL)
					fprintf(pFileAgentPathLog, "%d,%d,%d,%f,", p_agent->agent_id, p_agent->origin_node_id, p_agent->destination_node_id, m_node_label_cost[current_node_seq_no]);

				while (current_node_seq_no >= 0)
				{
					if (current_node_seq_no >= 0)  // this is valid node 
					{
						current_link_seq_no = m_link_predecessor[current_node_seq_no];

						if (current_link_seq_no >= 0)
						{
							p_agent->path_link_seq_no_vector.push_back(current_link_seq_no);
						}

						if (pFileAgentPathLog != NULL)
							fprintf(pFileAgentPathLog, "%d;", g_node_vector[current_node_seq_no].external_node_id);

						p_agent->path_node_id_vector.push_back(current_node_seq_no);

					}


					current_node_seq_no = m_node_predecessor[current_node_seq_no];


				}
				if (p_agent->fixed_path_flag != 1)
				{
					std::reverse(std::begin(p_agent->path_node_id_vector),
						std::end(p_agent->path_node_id_vector));

					std::reverse(std::begin(p_agent->path_link_seq_no_vector),
						std::end(p_agent->path_link_seq_no_vector));
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

	int TotalCumulative_Arrival_Count = 0;
	int TotalCumulative_Departure_Count = 0;

	for (int l = 0; l < g_number_of_links; l++)
	{
		g_link_vector[l].AllocateMemory();
		g_link_vector[l].ResetMOE();

		g_link_vector[l].m_End_of_RedTime_in_simu_interval = g_start_simu_interval_no;

	}
	// simulation
	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		CAgent* p_agent = &(g_agent_vector[a]);
		p_agent->departure_time_in_min = p_agent->departure_time_in_min;
		p_agent->AllocateMemory();
	}

	list<CActiveAgentElement> g_active_agent_pointer_list;  // ready to active, and still in the network
	int current_active_agent_id = 0;

	int number_of_threads = omp_get_max_threads();// the number of threads is redifined.


	for (int t = g_start_simu_interval_no; t < g_end_simu_interval_no; t++)  // first loop for time t
	{
		int number_of_simu_interval_per_min = 60 / g_number_of_seconds_per_interval;
		if (t % number_of_simu_interval_per_min == 0)
			cout << "simu time= " << t / number_of_simu_interval_per_min << " min, with TBL=" << g_active_agent_pointer_list.size() << ", CA = " << TotalCumulative_Arrival_Count << " CD=" << TotalCumulative_Departure_Count << endl;
		int relative_t = g_A2R_simu_interval(t);

		if (t % number_of_simu_interval_per_min == 0)//update the agent pointer list every minute
		{

#pragma omp parallel for 
			for (int a = current_active_agent_id; a < g_agent_vector.size(); a++)
			{
				CAgent* p_agent = &(g_agent_vector[a]);
				if (t <= p_agent->departure_time_in_simu_interval && p_agent->departure_time_in_simu_interval < t + number_of_simu_interval_per_min)  // PER MIN
				{
					p_agent->m_bGenereated = true;
					TotalCumulative_Arrival_Count++;
					CActiveAgentElement element;
					element.pAgent = p_agent;
					element.bActive = true;

					g_active_agent_pointer_list.push_back(element);

					current_active_agent_id = a + 1; //move to the next fresh agent id as active id
				}
				else  // late departure time
				{
					break;
				}
			}

		}


#pragma omp parallel for 
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)  // virutal loop for different processors
		{

			for (auto it = g_active_agent_pointer_list.begin(); it != g_active_agent_pointer_list.end(); ++it) // second loop for a
			{
				CAgent* p_agent = (*it).pAgent;

				if (p_agent->m_bGenereated == true &&
					p_agent->m_bCompleteTrip == false &&
					p_agent->m_bMoveable == true /* vehicle or walking pax */ &&
					p_agent->m_current_link_seq_no%number_of_threads == ProcessID &&
					p_agent->m_current_link_seq_no < p_agent->m_path_link_seq_no_vector_size
					)
				{
					if (p_agent->agent_id == 4 && p_agent->m_current_link_seq_no >= 0)
					{
						TRACE("");
					}


					if (p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no] == t)  // ready to move to the next link
					{
						// check if the current link has sufficient capacity 
						int link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no];

						/*condition 1:  moving link*/	if (g_link_vector[link_seq_no].service_type == 0)
						{


							if (g_link_vector[link_seq_no].m_End_of_RedTime_in_simu_interval <= t)
							{
								if (p_agent->m_current_link_seq_no == p_agent->path_link_seq_no_vector.size() - 1)
								{// end of path

									p_agent->m_bCompleteTrip = true;
									(*it).bActive = false;  // mark inactive element in the queue
									g_link_vector[link_seq_no].m_LinkCumulativeDeparture[relative_t] += 1;
									TotalCumulative_Departure_Count += 1;

								}
								else
								{ // not complete the trip

									int next_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];

									p_agent->m_Veh_LinkArrivalTime_in_simu_interval[p_agent->m_current_link_seq_no + 1] = t;
									p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no + 1] = t + g_link_vector[next_link_seq_no].free_flow_travel_time_in_simu_interval;

									for (auto i = p_agent->m_PassengerList.begin(); i != p_agent->m_PassengerList.end(); i++)
									{
										int p = (*i);  // obtain pax id
										int p_seq_no = g_map_agent_id_to_agent_vector_seq_no[p];
										CAgent* p_pax = &(g_agent_vector[p_seq_no]);

										p_pax->m_Veh_LinkDepartureTime_in_simu_interval[p_pax->m_current_link_seq_no] = t;  // current link's TD
										p_pax->m_Veh_LinkArrivalTime_in_simu_interval[p_pax->m_current_link_seq_no + 1] = t; // next link's TA
										p_pax->m_Veh_LinkDepartureTime_in_simu_interval[p_pax->m_current_link_seq_no + 1] = p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no + 1];  // copy value of TD on next link from vehicle
										p_pax->m_current_link_seq_no += 1;  // move to next link of pax too

									}

									g_link_vector[link_seq_no].m_CumulativeDepartureCount += 1;
									g_link_vector[link_seq_no].m_LinkCumulativeDeparture[relative_t] = g_link_vector[link_seq_no].m_CumulativeDepartureCount;
#pragma omp critical
									{
										g_link_vector[next_link_seq_no].m_CumulativeArrivalCount += 1;
										g_link_vector[next_link_seq_no].m_LinkCumulativeArrival[relative_t] = g_link_vector[next_link_seq_no].m_CumulativeArrivalCount;


									}

								}

								//move
								p_agent->m_current_link_seq_no += 1;

								//g_link_vector[link_seq_no].m_LinkOutFlowCapacity[relative_t] -= 1;

								int headway_in_simu_interval = max(1, int(1.0 / g_link_vector[link_seq_no].m_LinkOutFlowCapacity[relative_t] + 0.5));
								g_link_vector[link_seq_no].m_End_of_RedTime_in_simu_interval = t + headway_in_simu_interval;


							}
							else  // no outflow capacity ==0, cause delay
							{
								g_link_vector[link_seq_no].m_CumulativeVirtualDelayCount += 1 * g_number_of_seconds_per_interval / 60.0; // +1 means add one unit of simulation time interval of delay: unit: min
								g_link_vector[link_seq_no].m_LinkCumulativeVirtualDelay[relative_t] = g_link_vector[link_seq_no].m_CumulativeVirtualDelayCount;

								p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no] = t + 1;
							}
						}

						/*condition 2:  dropoff link*/	if (g_link_vector[link_seq_no].service_type == -1)
						{
							int next_vehicle_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];

							p_agent->m_Veh_LinkArrivalTime_in_simu_interval[p_agent->m_current_link_seq_no + 1] = t;
							p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no + 1] = t + g_link_vector[next_vehicle_link_seq_no].free_flow_travel_time_in_simu_interval;

							for (auto i = p_agent->m_PassengerList.begin(); i != p_agent->m_PassengerList.end();)
							{
								int p = (*i);  // obtain pax id
								int p_seq_no = g_map_agent_id_to_agent_vector_seq_no[p];
								CAgent* p_pax = &(g_agent_vector[p_seq_no]);

								int next_pax_link_seq_no = p_pax->path_link_seq_no_vector[p_pax->m_current_link_seq_no + 1];

								p_pax->m_Veh_LinkDepartureTime_in_simu_interval[p_pax->m_current_link_seq_no] = t;  // current link's TD
								p_pax->m_Veh_LinkArrivalTime_in_simu_interval[p_pax->m_current_link_seq_no + 1] = t; // next link's TA
								p_pax->m_Veh_LinkDepartureTime_in_simu_interval[p_pax->m_current_link_seq_no + 1] = p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no + 1];  // copy value of TD on next link from vehicle. It seems that it add the free-flow travel time of next moving link of passgers rather than the vehilce's
								p_pax->m_current_link_seq_no += 1;  // move to next link of pax too

								if (next_pax_link_seq_no != next_vehicle_link_seq_no)// their next link seq no is not consistent
								{  // drop off pax remove it from the list
									i = p_agent->m_PassengerList.erase(i);

									//update pax's TA and TD
									p_pax->m_bMoveable = true;

								}
								else
								{
									++i;
								}

							}
							//move
							p_agent->m_current_link_seq_no += 1;
							g_link_vector[link_seq_no].m_LinkOutFlowCapacity[relative_t] -= 1;

						}

						/*condition 3:  pick up link*/	if (g_link_vector[link_seq_no].service_type == 1)
						{
							if (p_agent->agent_service_type == 1 && p_agent->m_bMoveable == 1)  // pax: can enter link waiting queue
							{
								g_link_vector[link_seq_no].m_waiting_traveler_queue.push_back(p_agent->agent_id);

							}

							if (p_agent->agent_service_type == 2)  // vehicle: can pick up travelers from the link waiting queue
							{
								int remaining_seat_capacity = p_agent->GetRemainingCapacity();
								for (int p_ready = 0; p_ready < remaining_seat_capacity; p_ready++)
								{
									if (g_link_vector[link_seq_no].m_waiting_traveler_queue.size() == 0)
									{
										break;// no traveler in queue, no need to continue
									}

									int p = g_link_vector[link_seq_no].m_waiting_traveler_queue.front();
									g_link_vector[link_seq_no].m_waiting_traveler_queue.pop_front();
									p_agent->Pickup(p);

								}  // end of pick up process

								int next_vehicle_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];

								p_agent->m_Veh_LinkArrivalTime_in_simu_interval[p_agent->m_current_link_seq_no + 1] = t;
								p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no + 1] = t + g_link_vector[next_vehicle_link_seq_no].free_flow_travel_time_in_simu_interval;

								for (auto i = p_agent->m_PassengerList.begin(); i != p_agent->m_PassengerList.end(); i++)
								{
									int p = (*i);  // obtain pax id

									int p_seq_no = g_map_agent_id_to_agent_vector_seq_no[p];
									CAgent* p_pax = &(g_agent_vector[p_seq_no]);


									// move all pax's timetable to the next link along the vehicle trajectory
									int next_pax_link_seq_no = p_pax->path_link_seq_no_vector[p_pax->m_current_link_seq_no + 1];

									p_pax->m_Veh_LinkDepartureTime_in_simu_interval[p_pax->m_current_link_seq_no] = t;  // current link's TD
									p_pax->m_Veh_LinkArrivalTime_in_simu_interval[p_pax->m_current_link_seq_no + 1] = t; // next link's TA
									p_pax->m_Veh_LinkDepartureTime_in_simu_interval[p_pax->m_current_link_seq_no + 1] = p_agent->m_Veh_LinkDepartureTime_in_simu_interval[p_agent->m_current_link_seq_no + 1];  // copy value of TD on next link from vehicle
									p_pax->m_current_link_seq_no += 1;  // move to next link of pax too
									p_pax->m_bMoveable = false;
								}

								//move
								p_agent->m_current_link_seq_no += 1;
								g_link_vector[link_seq_no].m_LinkOutFlowCapacity[relative_t] -= 1;

							}

						} // conditions


					}  // departure time events
				}  // active agent

			}
		} // agent pointers

			// clean up list 

		for (auto i = g_active_agent_pointer_list.begin(); i != g_active_agent_pointer_list.end();)
		{
			if ((*i).bActive == false)
				i = g_active_agent_pointer_list.erase(i);
			else
				++i;
		}
	}
}  // end for processor



void g_AssignThreadsBasedonCPUCores()
{

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
}

int g_AssignThreadsBasedonOriginNodes()
{

	// simulation
	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		CAgent* p_agent = &(g_agent_vector[a]);

		if (p_agent->fixed_path_flag != 1)
		{
			int internal_origin_node_seq_no = g_internal_node_seq_no_map[p_agent->origin_node_id];  // map external node number to internal node seq no. 
			g_node_vector[internal_origin_node_seq_no].bOriginNode_ForAgents = true;
		}

	}

	int number_of_origin_nodes = 0;
	for (int i = 0; i < g_node_vector.size(); i++)
	{
		if (g_node_vector[i].bOriginNode_ForAgents == true)
		{
			g_node_vector[i].m_OriginNodeSeqNo = number_of_origin_nodes;
			number_of_origin_nodes++;
		}

	}

	int number_of_threads = max(1, number_of_origin_nodes);
	pNetworkForSP = new NetworkForSP[number_of_threads]; // create n copies of network, each for a subset of agents to use 

	cout << " number of threads = " << number_of_threads << endl;
	for (int i = 0; i < number_of_threads; i++)
	{
		pNetworkForSP[i].m_threadNo = i;   // each thread/network has its own thread number. // each network has its own label cost vector for shortest path calculation
		pNetworkForSP[i].AllocateMemory(g_number_of_nodes, g_number_of_links);
	}
	cout << " end of network memory allocation. " << endl;

	number_of_origin_nodes = 0;
	for (int i = 0; i < g_node_vector.size(); i++)
	{
		if (g_node_vector[i].bOriginNode_ForAgents == true)
		{
			pNetworkForSP[number_of_origin_nodes].m_private_origin_seq_no = g_node_vector[i].node_seq_no;

			number_of_origin_nodes++;
		}

	}

	for (int a = 0; a < g_agent_vector.size(); a++)  //assign all agents to the corresponding thread
	{
		CAgent* p_agent = &(g_agent_vector[a]);
		if (p_agent->fixed_path_flag != 1)
		{
			int internal_origin_node_seq_no = g_internal_node_seq_no_map[p_agent->origin_node_id];  // map external node number to internal node seq no. 

			int thread_no = g_node_vector[internal_origin_node_seq_no].m_OriginNodeSeqNo;
			pNetworkForSP[thread_no].m_agent_vector.push_back(a);
		}
	}
	return max(1, number_of_threads);
}

// definte global timestamps
clock_t start_t, end_t, total_t;

void g_TrafficAssignment()
{
	int i;

	start_t = clock();

	g_AssignThreadsBasedonCPUCores();

	int number_of_threads = g_AssignThreadsBasedonOriginNodes();// the number of threads could be a large number if there are many origins

	for (int assignment_iteration_no = 0; assignment_iteration_no < 2; assignment_iteration_no++)
	{
		cout << "Assignment Interval = %d" << assignment_iteration_no << endl;



#pragma omp parallel for  // step 3: C++ open mp automatically create n threads., each thread has its own computing thread on a cpu core 
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)// could the computer have that many processors?
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

			}
		}

		for (int l = 0; l < g_number_of_links; l++)
		{
			g_link_vector[l].CalculateBPRFunctionAndCost();  //based on the g_link_vector[l].flow_volume
			fprintf(g_pFileDebugLog, "\iteration = %d, link no= %d, volume = %f, travel_time = %f\n",
				assignment_iteration_no, g_link_vector[l].flow_volume, g_link_vector[l].travel_time);
		}

	}


}


void g_OutputTDLinkMOE()
{
	FILE* st = NULL;
	st = fopen("output_LinkTDMOE.csv", "w");
	if (st == NULL)
	{
		cout << "File output_LinkTDMOE.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//parser.GetValueByFieldName("exit_queue_length", pLink->m_LinkMOEAry[t].QueueLength);
	//parser.GetValueByFieldName("number_of_queued_vehicles", pLink->m_LinkMOEAry[t].number_of_queued_vehicles);

	fprintf(st, "from_node_id,to_node_id,timestamp_in_min,travel_time_in_min,link_service_vehicle_count,LR_price,link_volume_in_veh_per_hour_for_all_lanes,density_in_veh_per_distance_per_lane,speed,cumulative_arrival_count,cumulative_departure_count\n");

	for (int l = 0; l < g_link_vector.size(); l++) //Initialization for all nodes
	{
		CLink* pLink = &(g_link_vector[l]);

		for (int t = g_Simulation_StartTimeInMin; t < g_Simulation_EndTimeInMin; t++)
		{
			int time_in_simu_interval = t * 60 / g_number_of_seconds_per_interval;

			float link_count = pLink->get_link_in_flow_per_min(t)*60.0;

			int link_service_vehicle_count = g_link_vector[l].time_dependent_visit_counts[t];
			float LR_price = g_link_vector[l].time_dependent_LR_multiplier_vector[t];

			float avg_travel_time_in_min = pLink->free_flow_travel_time_in_min + pLink->get_avg_delay_in_min(t, 1);
			fprintf(st, "%d,%d,%d,%.3f,%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
				g_node_vector[pLink->from_node_seq_no].external_node_id,
				g_node_vector[pLink->to_node_seq_no].external_node_id,
				t,
				avg_travel_time_in_min,
				link_service_vehicle_count,
				link_count,
				LR_price,
				pLink->get_number_of_vehicles(t) / max(0.0001, pLink->length * pLink->number_of_lanes),
				pLink->length / max(0.001, avg_travel_time_in_min) / 60.0,
				pLink->m_LinkCumulativeArrival[g_A2R_simu_interval(time_in_simu_interval)],
				pLink->m_LinkCumulativeDeparture[g_A2R_simu_interval(time_in_simu_interval)]);
		}

	}

	fclose(st);
}


void g_OutputFiles()
{
	cout << "outputing files... " << endl;

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
				g_node_vector[pLink->from_node_seq_no].external_node_id,
				g_node_vector[pLink->to_node_seq_no].external_node_id,
				pLink->flow_volume,
				pLink->get_VOC_ratio(),
				pLink->get_speed(),
				pLink->travel_time);
		}


		fclose(g_pFileLinkMOE);
	}

	g_OutputTDLinkMOE();
	FILE* g_pFileAgent = NULL;
	g_pFileAgent = fopen("output_agent.csv", "w");
	if (g_pFileAgent == NULL)
	{
		cout << "File output_agent.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	else
	{

		fprintf(g_pFileAgent, "agent_id,agent_service_type,origin_node_id,destination_node_id,cost,departure_time_in_min,path_node_sequence,path_link_sequence,path_load_sequence,path_time_sequence,path_time_sequence_in_sec\n");

		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			fprintf(g_pFileAgent, "%d,%d,%d,%d,%f,%.1f,",
				p_agent->agent_id,
				p_agent->agent_service_type,
				p_agent->origin_node_id,
				p_agent->destination_node_id,
				p_agent->path_cost,
				p_agent->departure_time_in_min
			);

			// path node id sequence
			for (int i = 0; i < p_agent->path_node_id_vector.size(); i++)
			{
				/*int internal_node_id = p_agent->path_node_id_vector[i];
				int external_node_id = g_node_vector[internal_node_id].external_node_id;*/
				int external_node_id = p_agent->path_node_id_vector[i];
				fprintf(g_pFileAgent, "%d;", external_node_id);
			}

			fprintf(g_pFileAgent, ",");
			// path link id sequence 
			for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
			{
				int internal_link_id = p_agent->path_link_seq_no_vector[i];
				int external_link_id = g_link_vector[internal_link_id].external_link_id;

				if (external_link_id >= 1)
				{
					fprintf(g_pFileAgent, "%d;", external_link_id);
				}
			}
			fprintf(g_pFileAgent, ",");

			for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
			{
				int internal_link_id = p_agent->path_link_seq_no_vector[i];
				int load_id = g_link_vector[internal_link_id].VRP_load_id;

				if (load_id >= 1)
				{
					fprintf(g_pFileAgent, "%d;", load_id);
				}
			}
			fprintf(g_pFileAgent, ",");

			if (p_agent->path_timestamp_vector.size() >= 1)
			{
				// path node id sequence
				for (int i = 0; i < p_agent->path_timestamp_vector.size(); i++)
				{
					fprintf(g_pFileAgent, "%d;", p_agent->path_timestamp_vector[i]);
				}
			}
			else
			{

				if (p_agent->m_Veh_LinkArrivalTime_in_simu_interval != NULL && p_agent->m_Veh_LinkDepartureTime_in_simu_interval != NULL)
				{
					for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
					{

						float TA_in_min = p_agent->m_Veh_LinkArrivalTime_in_simu_interval[i] * g_number_of_seconds_per_interval / 60.0;
						float TD_in_min = p_agent->m_Veh_LinkDepartureTime_in_simu_interval[i] * g_number_of_seconds_per_interval / 60.0;

						if (i == 0)  // first link
							fprintf(g_pFileAgent, "%.3f:%.3f;", TA_in_min, TD_in_min);
						else
							fprintf(g_pFileAgent, "%.3f;", TD_in_min);
					}
				}


				if (p_agent->m_Veh_LinkArrivalTime_in_simu_interval != NULL && p_agent->m_Veh_LinkDepartureTime_in_simu_interval != NULL)
				{
					for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
					{

						float TA_in_sec = g_A2R_simu_interval(p_agent->m_Veh_LinkArrivalTime_in_simu_interval[i]) * g_number_of_seconds_per_interval;
						float TD_in_sec = g_A2R_simu_interval(p_agent->m_Veh_LinkDepartureTime_in_simu_interval[i]) * g_number_of_seconds_per_interval;


						if (i == 0)  // first link
							fprintf(g_pFileAgent, "%.2f:%.2f;", TA_in_sec, TD_in_sec);
						else
							fprintf(g_pFileAgent, "%.2f;", TD_in_sec);

					}
				}
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

int g_state_to_load_mapping[_MAX_STATES];
int g_initial_state_no = 0;   // customized 

void g_SetupStateMapping()
{
	g_state_to_load_mapping[0] = 0;
	g_state_to_load_mapping[1] = 1;
	g_state_to_load_mapping[2] = 2;
	g_state_to_load_mapping[3] = 3;

	g_initial_state_no = 0;
}

int g_from_load_to_state_mapping(int load)
{
	int state;

	state = load;

	return state;
}


//class for vehicle scheduling states
class CVSState
{
public:

	int current_node_id;  // space dimension
						  //passengerID, nodeType 
	int current_time_t;
	int current_link_no;  // from which to visit the current node

	int pred_node_id;
	int pred_time_t;
	int pred_state_w_index;

	std::map<int, int> passenger_service_state;
	std::map<int, int> passenger_service_end_time;
	std::map<int, int> passenger_service_begin_time;
	std::map<int, int> passenger_carrying_state;

	int m_vehicle_remaining_capacity;
	float LabelCost;  // with LR price
	float PrimalLabelCost;  // without LR price

	int m_final_arrival_time;   // for ending states
	CVSState()
	{
		pred_node_id = -1;
		pred_time_t = -1;
		pred_state_w_index = -1;

		current_time_t = -1;
		current_link_no = -1;
		m_final_arrival_time = 0;
		LabelCost = _MAX_LABEL_COST;
		m_vehicle_remaining_capacity = 0;
	}

	void Copy(CVSState* pSource)
	{
		current_node_id = pSource->current_node_id;
		passenger_service_state.clear();
		passenger_service_state = pSource->passenger_service_state;

		passenger_service_end_time.clear();
		passenger_service_end_time = pSource->passenger_service_end_time;

		passenger_service_begin_time.clear();
		passenger_service_begin_time = pSource->passenger_service_begin_time;

		passenger_carrying_state.clear();
		passenger_carrying_state = pSource->passenger_carrying_state;

		m_vehicle_remaining_capacity = pSource->m_vehicle_remaining_capacity;
		LabelCost = pSource->LabelCost;
	}
	int GetPassengerServiceState(int passenger_id)
	{
		if (passenger_service_state.find(passenger_id) != passenger_service_state.end())
			return passenger_service_state[passenger_id];  // 1 or 2
		else
			return 0;
	}

	void StartCarryingService(int passenger_id, int service_time)
	{
		passenger_carrying_state[passenger_id] = 1;
		m_vehicle_remaining_capacity -= 1;

		passenger_service_begin_time[passenger_id] = service_time;
	}

	void CompleteCarryingService(int passenger_id, int service_time)
	{
		map<int, int>::iterator iter = passenger_carrying_state.find(passenger_id);
		if (iter != passenger_carrying_state.end())
		{
			passenger_carrying_state[passenger_id] = 2;
			m_vehicle_remaining_capacity += 1;
		}
		passenger_service_end_time[passenger_id] = service_time;
	}

	//Start or Complete service
	void MarkCarryingService(int passenger_id, int link_service_type, int ServiceTime)
	{
		if (link_service_type == 1)
			StartCarryingService(passenger_id, ServiceTime);
		if (link_service_type == -1)
			CompleteCarryingService(passenger_id, ServiceTime);
	}

	std::string generate_string_key_version2()
	{
		stringstream s;

		s << " ";

		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			s << "_";

			s << it->first << "[" << it->second << "]";

		}
		string converted(s.str());
		return converted;

	}

	std::string generate_string_key()
	{
		stringstream s;
		s << m_vehicle_remaining_capacity;

		string converted(s.str());
		return converted;

	}

	bool operator<(const CVSState &other) const
	{
		return LabelCost < other.LabelCost;
	}

};

class C_time_indexed_state_vector
{

public:
	int current_time;

	std::vector<CVSState> m_VSStateVector;
	//state string 1_1_1,state index 
	std::map<std::string, int> m_state_map;

	void Reset()
	{
		current_time = 0;
		m_VSStateVector.clear();
		m_state_map.clear();
	}

	int m_find_state_index(std::string string_key)
	{

		if (m_state_map.find(string_key) != m_state_map.end())
		{
			return m_state_map[string_key];
		}
		else
			return -1;  // not found

	}

	void update_state(CVSState new_to_element)
	{
		std::string string_key = new_to_element.generate_string_key();//if it is new, string is n100, no state index
		int state_index = m_find_state_index(string_key);

		if (state_index == -1)  // no such state at this time
		{
			// add new state
			state_index = m_VSStateVector.size();
			m_VSStateVector.push_back(new_to_element);
			m_state_map[string_key] = state_index;
		}
		else
		{//DP 
			if (new_to_element.LabelCost < m_VSStateVector[state_index].LabelCost)
			{
				m_VSStateVector[state_index].Copy(&new_to_element);
			}

		}

	}

	void Sort()
	{
		std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

		m_state_map.clear(); // invalid
	}

	void SortAndCleanEndingState(int BestKValue)
	{
		if (m_VSStateVector.size() > 2 * BestKValue)
		{
			std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

			m_state_map.clear(); // invalid
			m_VSStateVector.erase(m_VSStateVector.begin() + BestKValue, m_VSStateVector.end());
		}
	}

	float GetBestValue(int DualPriceFlag, int vehicle_id)
	{
		// LabelCost not PrimalCost when sorting
		std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

		if (m_VSStateVector.size() >= 1)
		{
			std::string state_str = m_VSStateVector[0].generate_string_key();

			if (DualPriceFlag == 1)
				return m_VSStateVector[0].LabelCost;
			else
				return m_VSStateVector[0].PrimalLabelCost;
		}
		else
			return _MAX_LABEL_COST;
	}

};


class STSNetwork  // mainly for STS shortest path calculation
{
public:
	int m_threadNo;  // internal thread number 
	std::vector<int>  m_agent_vector; // assigned agents for computing 

	int m_number_of_nodes, m_number_of_time_intervals, m_number_of_states;

	int m_origin_node;
	int m_departure_time_beginning;
	int m_arrival_time_ending;

	float*** m_label_cost;
	float*** m_physical_label_cost;
	int*** m_node_predecessor;
	int*** m_time_predecessor;
	int*** m_state_predecessor;

	int*** m_pax_id_record;  // the memory of visted pax id



	int m_memory_allocation_method;
	//vehicle state at time t
	C_time_indexed_state_vector** m_time_dependent_state_vector;
	STSNetwork()
	{
		m_origin_node = -1;
		m_label_cost = NULL;
		m_node_predecessor = NULL;
		m_time_predecessor = NULL;
		m_state_predecessor = NULL;

		m_pax_id_record = NULL;
	}

	void AllocateSTSMemory(int number_of_nodes, int number_of_time_intervals, int number_of_states, int memory_allocation_method = 0)
	{
		m_memory_allocation_method = memory_allocation_method;  // static =0 vs dynamic = 1
		m_number_of_nodes = number_of_nodes;
		m_number_of_time_intervals = number_of_time_intervals;
		m_number_of_states = number_of_states;

		if (memory_allocation_method == 0)
		{
			m_label_cost = Allocate3DDynamicArray<float>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
			m_physical_label_cost = Allocate3DDynamicArray<float>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
			m_node_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
			m_time_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
			m_state_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
			m_pax_id_record = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		}
		else
		{
			m_time_dependent_state_vector = AllocateDynamicArray<C_time_indexed_state_vector>(g_node_vector.size(), g_number_of_optimization_time_intervals);
		}
	}

	~STSNetwork()
	{
		Deallocate3DDynamicArray<float>(m_label_cost, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<float>(m_physical_label_cost, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_node_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_time_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_state_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_pax_id_record, m_number_of_nodes, m_number_of_time_intervals);
	}

	void ADMM_penalty_method(int this_agent_id)
	{
		// step 1: reset
		for (int link_no = 0; link_no < g_link_vector.size(); link_no++)
		{
			for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
			{
				g_link_vector[link_no].time_dependent_ADMM_visit_counts[t] = 0;
			}
		}

		// step 2: scan all the other agents
		for (int j = 0; j < g_agent_vector.size(); j++)
		{
			CAgent* p_agent_other = &(g_agent_vector[j]);
			if (p_agent_other->agent_id != this_agent_id)  // not the current agent. 
			{

				// loop through the map
				for (std::map<int, int>::iterator it = p_agent_other->m_VRP_ADMM_link_time_map.begin();
					it != p_agent_other->m_VRP_ADMM_link_time_map.end(); ++it)
				{
					int this_link_no = it->first;
					int this_time = it->second;
					g_link_vector[this_link_no].time_dependent_ADMM_visit_counts[this_time] += 1;

				}
			}
		}

		// step 3: calculate ADMM capacity penalty 
		for (int link_no = 0; link_no < g_link_vector.size(); link_no++)
		{
			for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
			{
				if (g_link_vector[link_no].service_type == enum_road_capacity_link_type)
				{
					int time_dependent_visit_counts_used_by_other_vehicles = g_link_vector[link_no].time_dependent_ADMM_visit_counts[t];

					g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t] = max(0,
						(g_penalty_RHO / 2.0)*(2 * time_dependent_visit_counts_used_by_other_vehicles - g_link_vector[link_no].time_depedent_capacity_vector[t]));

					if (time_dependent_visit_counts_used_by_other_vehicles >= 1)
					{
						//TRACE("link = %d->%d, time = %d, ADMM capacity multiplier = %f",
						//	g_node_vector[g_link_vector[link_no].from_node_seq_no].external_node_id,
						//	g_node_vector[g_link_vector[link_no].to_node_seq_no].external_node_id,
						//	t, g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t]);
						//TRACE("\n");
					}

				}

			}  // end for each t
		} //end for each link

		  // step 4: calculate ADMM demand penalty 
		for (int link_no = 0; link_no < g_link_vector.size(); link_no++)
		{

			if (g_link_vector[link_no].service_type == 1)   // pick up
			{  // visited once, then no one is allowed to visit this link anymore
				int time_dependent_visit_counts_used_by_other_vehicles = 0;

				for (int t = g_link_vector[link_no].VRP_time_window_begin; t <= g_link_vector[link_no].VRP_time_window_end; t++)
				{
					time_dependent_visit_counts_used_by_other_vehicles += g_link_vector[link_no].time_dependent_ADMM_visit_counts[t];
				}

				// across all the timestamps in the time window
				for (int t = g_link_vector[link_no].VRP_time_window_begin; t <= g_link_vector[link_no].VRP_time_window_end; t++)
				{

					g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t] =
						(g_penalty_RHO / 2.0)*(2 * time_dependent_visit_counts_used_by_other_vehicles - 1);

					if (time_dependent_visit_counts_used_by_other_vehicles >= 1)
					{
						TRACE("link = %d->%d, time = %d, ADMM capacity multiplier = %f",
							g_node_vector[g_link_vector[link_no].from_node_seq_no].external_node_id,
							g_node_vector[g_link_vector[link_no].to_node_seq_no].external_node_id,
							t, g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t]);
						TRACE("\n");
					}

				}  // end for each t

			}  // if condition for demand link

		} //end for each link
	}

	//parallel computing version
	float optimal_STS_dynamic_programming(int VRP_group_id, int departure_time_beginning, int arrival_time_ending, int optimization_method)
	{
		if (m_origin_node < 0)
			return -1;

		std::map<int, int> node_scan_eligible_list_flag;

		float total_cost = _MAX_LABEL_COST;

		if (g_node_vector[m_origin_node].m_outgoing_node_vector.size() == 0)
		{
			return _MAX_LABEL_COST;
		}

		if (arrival_time_ending > m_number_of_time_intervals - 1)
		{
			return _MAX_LABEL_COST;
		}

		CAgent* p_agent = &(g_agent_vector[m_agent_vector[0]]);

		// step 1: Initialization for all nodes
		for (int i = 0; i < m_number_of_nodes; i++)
		{
			// to do: only update node label on the agent path
			for (int t = departure_time_beginning; t <= arrival_time_ending; t++)
			{
				for (int w = 0; w <= p_agent->vehicle_seat_capacity; w++)
				{
					m_label_cost[i][t][w] = _MAX_LABEL_COST;
					m_physical_label_cost[i][t][w] = _MAX_LABEL_COST;
					m_node_predecessor[i][t][w] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
					m_time_predecessor[i][t][w] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
					m_state_predecessor[i][t][w] = -1;
					m_pax_id_record[i][t][w] = -1;
				}
			}
		}

		//step 2: Initialization for origin node at the preferred departure time, at departure time

		int w0 = g_initial_state_no;  // start from empty

		for (int t = departure_time_beginning; t < min(arrival_time_ending, g_number_of_optimization_time_intervals - 1); t++)  //first loop: time
		{

			//			m_label_cost[m_origin_node][t][w0] = t- departure_time_beginning;  // waiting at origin
			m_label_cost[m_origin_node][t][w0] = t - departure_time_beginning;  // waiting at origin
			m_physical_label_cost[m_origin_node][t][w0] = t - departure_time_beginning;
		}
		node_scan_eligible_list_flag[m_origin_node] = 1;

		//CAgent* p_agent = &(g_agent_vector[m_agent_vector[0]]); //the first agent is the current agent

		// step 3: //dynamic programming , absoluate time
		if (g_dp_algorithm_debug_flag == 1)
		{
			fprintf(g_pFileDebugLog, "****************starting of the DP****************\n");
			fprintf(g_pFileDebugLog, "agent_id = %d\n", p_agent->agent_id);
		}

		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
		{


			if (g_dp_algorithm_debug_flag == 1)
			{
				fprintf(g_pFileDebugLog, "t = %d\n", t);
			}

			for (std::map<int, int>::iterator it = node_scan_eligible_list_flag.begin(); it != node_scan_eligible_list_flag.end(); ++it)
			{

				int n = it->first;

				if (t == 32 && n == 8)
					TRACE("");

				for (int link = 0; link < g_node_vector[n].m_outgoing_node_vector.size(); link++)
				{
					int link_no = g_node_vector[n].m_outgoing_node_vector[link].link_seq_no;
					int from_node = g_node_vector[n].m_outgoing_node_vector[link].from_node_seq_no;
					int to_node = g_node_vector[n].m_outgoing_node_vector[link].to_node_seq_no;

					if (g_dp_algorithm_debug_flag == 1)
					{
						fprintf(g_pFileDebugLog, "link_id = %d, from_node = %d, to_node = %d\n", g_link_vector[link_no].link_seq_no, g_internal_node_seq_no_to_node_id_map[from_node],
							g_internal_node_seq_no_to_node_id_map[to_node]);
					}


					if (g_link_vector[link_no].service_type != 0)   // pick up or drop off
					{ // reset dynamically the waiting time
						if (t>  g_link_vector[link_no].VRP_time_window_end)
							continue;

						if (g_link_vector[link_no].VRP_group_id != VRP_group_id)
							continue;

					}


					for (int w1 = 0; w1 <= p_agent->vehicle_seat_capacity; w1++) // for each state
					{
						if (g_dp_algorithm_debug_flag == 1)
						{
							fprintf(g_pFileDebugLog, "w1 = %d\n", w1);
						}

						if (m_label_cost[from_node][t][w1] < _MAX_LABEL_COST - 1)  //for feasible time-space point only
						{
							int travel_time = 0;
							//										travel_time = p_agent->time_dependent_link_travel_time[link_no][t] + wait_time;
							travel_time = g_link_vector[link_no].free_flow_travel_time_in_min;  // to do: consider time-dependent travel time:L XS.
							int travel_cost = 0;

							int from_load_state = w1;

							int to_load_state = max(0, from_load_state + g_link_vector[link_no].VRP_load_difference);

							if (to_load_state > p_agent->vehicle_seat_capacity)// this is the vehicle capacity constraint
								continue;

							int w2 = to_load_state;

							int new_to_node_arrival_time = min(t + travel_time, g_number_of_optimization_time_intervals - 1); // plus the waiting time on the link

							if (new_to_node_arrival_time == t) // XUESONG: we do not allow update at the same time interval
								continue;

							if (g_link_vector[link_no].service_type != 0 && new_to_node_arrival_time < g_link_vector[link_no].VRP_time_window_begin)
							{
								new_to_node_arrival_time = g_link_vector[link_no].VRP_time_window_begin; // reset the arrival time to the service node, 
							}

							int passenger_id = g_link_vector[link_no].VRP_load_id;

							//if (passenger_id >= 1)  // service node
							//{
							//	for (int w1index = 1; w1index < w2; w1index++)
							//	{
							//		if (m_pax_id_record[from_node][t][w1index] == passenger_id)
							//		{  //
							//			travel_time = 999999; // prevent re-visit the same pax
							//		}
							//	}

							//}

							float temporary_label_cost = 0;
							float temporary_physical_label_cost = 0;

							float sum_of_multipliers = 0;

							if (g_dp_algorithm_debug_flag == 1)
							{
								fprintf(g_pFileDebugLog, "w2 = %d, travel_time = %d\n", w2, travel_time);
							}


							if (optimization_method == 0 || optimization_method == 2) //LR or ADMM
							{
								sum_of_multipliers += g_link_vector[link_no].time_dependent_LR_multiplier_vector[t];
							}
							else if (optimization_method == 1) // priority rule
							{
								sum_of_multipliers = 0;
							}


							if (optimization_method == 2) // ADMM
							{
								sum_of_multipliers += g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t];


							}

							temporary_label_cost = m_label_cost[from_node][t][w1]
								+ travel_time + sum_of_multipliers
								+ g_link_vector[link_no].time_dependent_external_cost_vector[t];
							
							temporary_physical_label_cost = m_physical_label_cost[from_node][t][w1]+ travel_time;
								
							if (temporary_label_cost < m_label_cost[to_node][new_to_node_arrival_time][w2]) // we only compare cost at the downstream node ToID at the new arrival time t
							{
								// update cost label and node/time predecessor
								m_label_cost[to_node][new_to_node_arrival_time][w2] = temporary_label_cost;
								m_physical_label_cost[to_node][new_to_node_arrival_time][w2] = temporary_physical_label_cost;
								m_node_predecessor[to_node][new_to_node_arrival_time][w2] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								m_time_predecessor[to_node][new_to_node_arrival_time][w2] = t;  // pointer to previous TIME INDEX from the current label at current node and time
								m_state_predecessor[to_node][new_to_node_arrival_time][w2] = w1;

							
								// copy the visited pax id from 0 to w1
								for (int w1index = 0; w1index <= w1; w1index++)
								{
									m_pax_id_record[to_node][new_to_node_arrival_time][w1index] = m_pax_id_record[from_node][t][w1index];
								}
								m_pax_id_record[to_node][new_to_node_arrival_time][w2] = passenger_id;  // take the record of pax id at position w2

								node_scan_eligible_list_flag[to_node] = 1;
								//TRACE( "n = %d, t=%d, w = %d, label cost = %f; from node %d, from time t=%d, from w=%d\n", 
								//	to_node, new_to_node_arrival_time, w2, temporary_label_cost,
								//	from_node, t, w1);

							}

						}  // for all states
					} //for each outgoing link
				}
			}
		} // for all time t

		return total_cost;// Tony: total_cost is not updated.
	}



	float find_STS_path_for_agents_assigned_for_this_thread(int number_of_threads, int assignment_iteration_no, int optimization_method)
	{
		std::vector<int> path_node_sequence, path_link_sequence, path_time_sequence, path_state_sequence;

		std::vector<float>	path_cost_sequence;

		float total_cost = _MAX_LABEL_COST;
		float total_physical_cost = _MAX_LABEL_COST;

		for (int i = 0; i < m_agent_vector.size(); i++)
		{
			CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

			//if (p_agent->agent_service_type == 2)
			{
				p_agent->path_link_seq_no_vector.clear();  // reset;
				p_agent->path_timestamp_vector.clear();
				p_agent->path_node_id_vector.clear();  // reset;


				if (optimization_method == 2)
				{
					ADMM_penalty_method(p_agent->agent_id);
				}

				if (this->m_memory_allocation_method == 0)  // static
				{
					// perform one to all STS shortest path
					int return_value = optimal_STS_dynamic_programming(p_agent->VRP_group_id,
						m_departure_time_beginning, m_arrival_time_ending, optimization_method);

					if (return_value == -1)
					{
						fprintf(g_pFileDebugLog, "agent %d with can not find destination node,\n", i);
						continue;
					}
					int current_node_seq_no;
					int current_link_seq_no;

					//step 4: back trace from the destination node to find the shortest path from shortest path tree 
					int destination_node_seq_no = g_internal_node_seq_no_map[p_agent->destination_node_id];

					int min_cost_time_index = m_arrival_time_ending;

					total_cost = m_label_cost[destination_node_seq_no][min_cost_time_index][0];

					int final_state = -1;

					for (int t = m_departure_time_beginning; t < m_arrival_time_ending; t++)
					{
						for (int w=0; w <= p_agent->vehicle_seat_capacity; w++)// Tony: to find the min_cost label and state, the state is not 0.
							if (m_label_cost[destination_node_seq_no][t][w] < total_cost)
							{
								min_cost_time_index = t;
								final_state = w;
								total_cost = m_label_cost[destination_node_seq_no][t][w];
								total_physical_cost= m_physical_label_cost[destination_node_seq_no][t][w];
							}
					}

					if (final_state == -1)
						return -1;

					p_agent->path_cost = total_physical_cost;

					// step 2: backtrack to the origin (based on node and time predecessors)
					int	node_size = 0;
					path_node_sequence.push_back(destination_node_seq_no); //record the first node backward, destination node
					path_time_sequence.push_back(min_cost_time_index);
					path_state_sequence.push_back(final_state);
					path_cost_sequence.push_back(m_physical_label_cost[destination_node_seq_no][min_cost_time_index][final_state]);

					node_size++;

					int pred_node = m_node_predecessor[destination_node_seq_no][min_cost_time_index][final_state];
					int pred_time = m_time_predecessor[destination_node_seq_no][min_cost_time_index][final_state];
					int pred_state = m_state_predecessor[destination_node_seq_no][min_cost_time_index][final_state];

					while (pred_node != -1) // scan backward in the predessor array of the shortest path calculation results
					{
						path_node_sequence.push_back(pred_node);
						path_time_sequence.push_back(pred_time);
						path_state_sequence.push_back(pred_state);
						path_cost_sequence.push_back(m_physical_label_cost[pred_node][pred_time][pred_state]);

						node_size++;

						//record current values of node and time predecessors, and update PredNode and PredTime

						int pred_node_record = pred_node;
						int pred_time_record = pred_time;
						int pred_state_record = pred_state;

						pred_node = m_node_predecessor[pred_node_record][pred_time_record][pred_state_record];
						pred_time = m_time_predecessor[pred_node_record][pred_time_record][pred_state_record];
						pred_state = m_state_predecessor[pred_node_record][pred_time_record][pred_state_record];
					}

					//reverse the node sequence 
					std::reverse(std::begin(path_node_sequence), std::end(path_node_sequence));
					std::reverse(std::begin(path_time_sequence), std::end(path_time_sequence));
					std::reverse(std::begin(path_state_sequence), std::end(path_state_sequence));
					std::reverse(std::begin(path_cost_sequence), std::end(path_cost_sequence));

					//for (int i = 0; i < path_node_sequence.size()-1; i++)  // for each node 
					//{
					//	int internal_from_node_seq_no = path_node_sequence[i];
					//	int internal_to_node_seq_no = path_node_sequence[i+1];
					//	long link_key = internal_from_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_to_node_seq_no;
					//	int link_no=g_link_key_to_seq_no_map[link_key];

					//	p_agent->path_link_seq_no_vector.push_back(link_no);
					//	int entrance_time = path_time_sequence[i];  // we visit this link from time pred_time_t to time t
					//	g_link_vector[link_no].time_dependent_visit_counts[entrance_time] += 1;
					//}

				}
				else  // dynamic memory
				{
					int BestKSize = 5;

					std::vector<CVSState*> path_CVS_state_sequence;
					int return_value = optimal_time_dependenet_dynamic_programming(
						m_origin_node,
						m_departure_time_beginning,
						m_arrival_time_ending,
						p_agent->vehicle_seat_capacity,
						optimization_method,
						BestKSize);


					p_agent->path_link_seq_no_vector.clear();  // reset;
					p_agent->path_timestamp_vector.clear();
					p_agent->path_node_id_vector.clear();  // reset;

					int current_node_seq_no;
					int current_link_seq_no;

					//step 4: back trace from the destination node to find the shortest path from shortest path tree 
					int destination_node_seq_no = g_internal_node_seq_no_map[p_agent->destination_node_id];
					m_time_dependent_state_vector[destination_node_seq_no][m_arrival_time_ending].Sort();


					int n = destination_node_seq_no;
					int t;
					float total_cost = 99999;

					CVSState* pElementBest = NULL;
					// step 2: scan the best k elements

					for (int t = m_departure_time_beginning; t <= m_arrival_time_ending; t++)
					{
						int state_vertex_size = min(BestKSize, m_time_dependent_state_vector[n][t].m_VSStateVector.size());

						for (int w_index = 0; w_index < state_vertex_size; w_index++)
						{
							CVSState* pElement = &(m_time_dependent_state_vector[n][t].m_VSStateVector[w_index]);

							if (pElement->LabelCost <= total_cost)
							{
								total_cost = pElement->LabelCost;

								pElementBest = pElement;
							}
						}

					}

					if (pElementBest == NULL)
						return NULL;
					//
					// step 2: backtrack to the origin (based on node and time predecessors)
					int	node_size = 0;
					path_CVS_state_sequence.push_back(pElementBest); //record the first node backward, destination node

					CVSState* pElement = pElementBest;

					node_size++;

					int pred_node = pElement->pred_node_id;
					int pred_time = pElement->pred_time_t;
					int pred_state_index = pElement->pred_state_w_index;

					while (pred_node != -1) // scan backward in the predessor array of the shortest path calculation results
					{
						pElement = &(m_time_dependent_state_vector[pred_node][pred_time].m_VSStateVector[pred_state_index]);

						path_CVS_state_sequence.push_back(pElement);
						node_size++;
						//record current values of node and time predecessors, and update PredNode and PredTime

						pred_node = pElement->pred_node_id;
						pred_time = pElement->pred_time_t;
						pred_state_index = pElement->pred_state_w_index;
					}

					//reverse the node sequence 
					std::reverse(std::begin(path_CVS_state_sequence), std::end(path_CVS_state_sequence));

					for (int i = 0; i < path_CVS_state_sequence.size(); i++)  // for each node 
					{
						pElement = path_CVS_state_sequence[i];

						p_agent->path_node_id_vector.push_back(g_internal_node_seq_no_to_node_id_map[pElement->current_node_id]);
						p_agent->path_timestamp_vector.push_back(pElement->current_time_t);

						if (i >= 1)  // for each link
						{
							int link_no = pElement->current_link_no;
							p_agent->path_link_seq_no_vector.push_back(link_no);
							int entrance_time = pElement->pred_time_t;  // we visit this link from time pred_time_t to time t
							g_link_vector[link_no].time_dependent_visit_counts[entrance_time] += 1;
						}
					}
					// output now is a set of vectors: node id, timestamp and link sequence no.

				}


				TRACE("assignment_iteration_no = %d, agent = %d\n", assignment_iteration_no, p_agent->agent_id);

				for (int i = 0; i < path_node_sequence.size(); i++)  // for each node 
				{
					p_agent->path_node_id_vector.push_back(g_internal_node_seq_no_to_node_id_map[path_node_sequence[i]]);
					p_agent->path_timestamp_vector.push_back(path_time_sequence[i]);
					TRACE("iteration = %d, agent = %d, node = %d, time = %d, cost = %f\n", assignment_iteration_no, p_agent->agent_id,
						p_agent->path_node_id_vector[i], p_agent->path_timestamp_vector[i], path_cost_sequence[i]);

				}
				TRACE("------\n");

				p_agent->m_VRP_ADMM_link_time_map.clear();

				for (int i = 0; i < path_node_sequence.size(); i++)  // for each node 
				{
					if (i < path_node_sequence.size() - 1)  // for each link
					{
						int link_no = g_GetLinkSeqNo(p_agent->path_node_id_vector[i], p_agent->path_node_id_vector[i + 1]);  // Xuesong Changed
						p_agent->path_link_seq_no_vector.push_back(link_no);
						int entrance_time = p_agent->path_timestamp_vector[i];
						g_link_vector[link_no].time_dependent_visit_counts[entrance_time] += 1;

						//if(g_link_vector[link_no].time_dependent_LR_multiplier_vector[entrance_time] )
						//TRACE("iteration = %d, agent = %d, link from node = %d, time = %d, %f\n", 
						//	assignment_iteration_no, p_agent->agent_id,
						//	p_agent->path_node_id_vector[i], p_agent->path_timestamp_vector[i]);


						p_agent->m_VRP_ADMM_link_time_map[link_no] = entrance_time;

						if (optimization_method == 1) // priority rule, mark the infeasible region
						{
							if (g_link_vector[link_no].service_type == enum_road_capacity_link_type)
							{
								if (g_link_vector[link_no].time_dependent_visit_counts[entrance_time] >= g_link_vector[link_no].time_depedent_capacity_vector[entrance_time])
								{
									g_link_vector[link_no].time_dependent_external_cost_vector[entrance_time] = _MAX_LABEL_COST;
								}
							}

							if (g_link_vector[link_no].service_type == 1)   // pick up
							{  // visited once, then no one is allowed to visit this link anymore
								for (int t = g_link_vector[link_no].VRP_time_window_begin; t <= g_link_vector[link_no].VRP_time_window_end; t++)
								{
									g_link_vector[link_no].time_dependent_external_cost_vector[t] = _MAX_LABEL_COST;
								}
							}
						}


					}
				}

			}

		}
		return total_cost;
	}

	float optimal_time_dependenet_dynamic_programming(
		int origin_node,
		int departure_time_beginning,
		int arrival_time_ending,
		int vehicle_capacity,
		//maximum choose
		int optimization_method,
		int BestKSize

	)
		// time-dependent label correcting algorithm with double queue implementation
	{

		if (arrival_time_ending > g_number_of_optimization_time_intervals)
		{
			return _MAX_LABEL_COST;
		}

		//step 2: Initialization for origin node at the preferred departure time, at departure time
		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
		{
			for (int i = 0; i < g_number_of_nodes; i++)
			{
				m_time_dependent_state_vector[i][t].Reset();
			}
		}

		std::map<int, int> node_scan_eligible_list_flag;

		CVSState element;  // first space time state element 

		element.current_node_id = origin_node;
		element.current_time_t = departure_time_beginning;
		element.m_vehicle_remaining_capacity = vehicle_capacity;
		m_time_dependent_state_vector[origin_node][departure_time_beginning].update_state(element);

		node_scan_eligible_list_flag[origin_node] = 1;

		// step 3: //dynamic programming
		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
		{
			int state_count = 0;

			for (std::map<int, int>::iterator it = node_scan_eligible_list_flag.begin(); it != node_scan_eligible_list_flag.end(); ++it)
			{

				int n = it->first;

				// step 1: sort m_VSStateVector by labelCost for scan best k elements in step2 
				m_time_dependent_state_vector[n][t].Sort();

				int state_vertex_size = min(BestKSize, m_time_dependent_state_vector[n][t].m_VSStateVector.size());

				// step 2: scan the best k elements
				for (int w_index = 0; w_index < state_vertex_size; w_index++)
				{
					CVSState* p_from_Element = &(m_time_dependent_state_vector[n][t].m_VSStateVector[w_index]);
					state_count++;

					int from_node = p_from_Element->current_node_id;//current_node_id is not node n?  // the same as node n
					ASSERT(from_node == n);

					// step 2.1 link from node to toNode
					for (int i = 0; i < g_node_vector[from_node].m_outgoing_node_vector.size(); i++)
					{

						int link_no = g_node_vector[from_node].m_outgoing_node_vector[i].link_seq_no;
						int to_node = g_link_vector[link_no].to_node_seq_no;

						int VRP_step_size = 1;

						if (g_link_vector[link_no].service_type != 0)   // pick up or drop off
						{ // reset dynamically the waiting time

							if (t < g_link_vector[link_no].VRP_time_window_begin || t> g_link_vector[link_no].VRP_time_window_end)
								continue;

							int passenger_id = g_link_vector[link_no].VRP_load_id;

							if (passenger_id < 0)
								continue;
						}


						int travel_time = g_link_vector[link_no].free_flow_travel_time_in_min;  // to do: consider time-dependent travel time:L XS.
						int next_time = t + travel_time;
						//				int next_time = max(g_node_timestamp[to_node], t + g_link_free_flow_travel_time[link_no]);					
						if (next_time > arrival_time_ending)  // out of time horizon bound
							continue;

						int passenger_id = -1;

						if (g_link_vector[link_no].service_type != 0)   // pick up or drop off link
						{

							if (next_time < g_link_vector[link_no].VRP_time_window_begin
								|| next_time > g_link_vector[link_no].VRP_time_window_end)
								continue;

							passenger_id = g_link_vector[link_no].VRP_load_id;



							if (g_link_vector[link_no].service_type == 1 && p_from_Element->GetPassengerServiceState(passenger_id) >= 1)
							{
								// has been picked up or drop off, then no need to pick up again
								continue;
							}

							if (g_link_vector[link_no].service_type == 1 && p_from_Element->m_vehicle_remaining_capacity >= 1)
							{
								continue;
							}

							if (g_link_vector[link_no].service_type == -1 && p_from_Element->GetPassengerServiceState(passenger_id) != 1)
							{
								// not has been picked up, then no need to drop off
								continue;
							}
						}

						CVSState new_to_element;

						new_to_element.Copy(p_from_Element);

						if (passenger_id >= 0)  // with service
						{
							new_to_element.MarkCarryingService(passenger_id, g_link_vector[link_no].service_type, next_time);
						}
						new_to_element.current_node_id = to_node;
						new_to_element.current_time_t = next_time;
						new_to_element.current_link_no = link_no;
						new_to_element.pred_node_id = n;
						new_to_element.pred_time_t = t;
						new_to_element.pred_state_w_index = w_index;

						float sum_of_multipliers = 0;
						if (optimization_method == 0 || optimization_method == 2) //LR or ADMM
						{
							sum_of_multipliers += g_link_vector[link_no].time_dependent_LR_multiplier_vector[t];
						}
						else if (optimization_method == 1) // priority rule
						{
							sum_of_multipliers = 0;
						}


						if (optimization_method == 2) // ADMM
						{
							sum_of_multipliers += g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t];


						}

						float sum_of_cost = 0;

						sum_of_cost = travel_time + sum_of_multipliers
							+ g_link_vector[link_no].time_dependent_external_cost_vector[t];

						new_to_element.LabelCost = p_from_Element->LabelCost + sum_of_cost;

						m_time_dependent_state_vector[to_node][next_time].update_state(new_to_element);

						node_scan_eligible_list_flag[to_node] = 1;

					}  // for each link

				}

			}

		} // for each time t
	}

};

STSNetwork* pSTSNetwork = NULL;
void g_LR_ResourceMatrix_Initialization()
{

	for (int l = 0; l < g_link_vector.size(); l++)
	{
		g_link_vector[l].Setup_State_Dependent_Data_Matrix(g_number_of_optimization_time_intervals);
	}

	//for (int s = 0; s < g_signal_vector.size(); s++)  //TODO
	//{
	//	g_signal_vector[s].Singal_Setup_State_Dependent_Data_Matrix();
	//}

	//initialization of sts network


}

//parameters of LR
int g_number_of_LR_iterations = 2;
int g_number_of_ADMM_iterations = 100;
int g_CurrentLRIterationNumber = 0;
int g_Number_Of_Iterations_With_Memory = 5;

float g_best_upper_bound = 99999;
float g_best_lower_bound = -99999;
float g_stepSize = 0;
float g_penalty_PHO = 2;
float g_minimum_subgradient_step_size = 0.1;


bool g_LR_Optimization(int Memeory_Allocation_Mode = 0)
{
	int number_of_threads = 1;

	g_LR_ResourceMatrix_Initialization();

	pSTSNetwork = new STSNetwork[number_of_threads]; // create n copies of network, each for a subset of agents to use 
	pSTSNetwork[0].AllocateSTSMemory(g_number_of_nodes, g_number_of_optimization_time_intervals, _MAX_STATES, Memeory_Allocation_Mode);

	int thread_no = 0;
	int optimization_method = 2;

	// 0: pure LR
	//1: priority rule

	cout << "Lagrangian Relaxation Optimization..." << endl;

	//loop for each LR iteration vehicle_seat_capacity
	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{

		cout << LR_iteration + 1 << "/" << g_number_of_LR_iterations << endl;

		g_CurrentLRIterationNumber = LR_iteration + 1;

		g_stepSize = 1.0f / (LR_iteration + 1.0f);

		//keep the minimum step size
		if (g_stepSize < g_minimum_subgradient_step_size)
		{
			g_stepSize = g_minimum_subgradient_step_size;
		}


		// step 1: calaculate LR multipliers
		for (int l = 0; l < g_link_vector.size(); l++)  // for each link
		{
			if (g_link_vector[l].service_price == enum_road_capacity_link_type)  // capacity constraint; Tony: always satisfy
			{

				for (int t = 0; t < g_number_of_optimization_time_intervals; t++) // for each t
				{
					int link_visit_counts = g_link_vector[l].time_dependent_visit_counts[t];

					g_link_vector[l].time_dependent_LR_multiplier_vector[t] = max(0, g_link_vector[l].time_dependent_LR_multiplier_vector[t] +
						g_stepSize * (link_visit_counts - g_link_vector[l].time_depedent_capacity_vector[t]));

					if (link_visit_counts > g_link_vector[l].time_depedent_capacity_vector[t])
					{
						TRACE("l= %d, t=%d, capacity LR = %f\n", l, t, g_link_vector[l].time_dependent_LR_multiplier_vector[t]);
					}
				}
			}
			/////
			if (g_link_vector[l].service_type == 1)   // pick up
			{
				int link_visit_counts = 0;
				for (int t = g_link_vector[l].VRP_time_window_begin; t <= g_link_vector[l].VRP_time_window_end; t++) // for each t
				{
					link_visit_counts += g_link_vector[l].time_dependent_visit_counts[t];

				}
				for (int t = g_link_vector[l].VRP_time_window_begin; t <= g_link_vector[l].VRP_time_window_end; t++) // for each t
				{


					g_link_vector[l].time_dependent_LR_multiplier_vector[t] = g_link_vector[l].time_dependent_LR_multiplier_vector[t] +
						g_link_vector[l].base_price*g_stepSize * (link_visit_counts - 1);  // Tony: relaxation for the equal constraint

					if (link_visit_counts == 0)
					{
						TRACE("l= %d, t=%d, demand LR = %f\n", l, t, g_link_vector[l].time_dependent_LR_multiplier_vector[t]);
					}
				}

			}

		}
		// step 2: reset time_dependent_visit_counts  =0 
		for (int l = 0; l < g_link_vector.size(); l++)
		{
			for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
			{
				g_link_vector[l].time_dependent_visit_counts[t] = 0;

			}
		}

		// reset local LR lower bound
		float LR_global_lower_bound = 0;
		float LR_total_delay_time_in_lower_bound = 0;
		float total_price = 0;
		// one can resort the agent vector according to the profit priority
		// step 3: find DP optimal solution for each agent 
		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			if (p_agent->agent_service_type != 1)  // service vehicle
			{

				cout << "agent id = " << p_agent->agent_id << endl;

				int internal_origin_node_seq_no = g_internal_node_seq_no_map[p_agent->origin_node_id];  // map external node number to internal node seq no. 
				float trip_price = 0;

				pSTSNetwork[thread_no].m_agent_vector.clear();
				pSTSNetwork[thread_no].m_origin_node = internal_origin_node_seq_no;
				pSTSNetwork[thread_no].m_departure_time_beginning = p_agent->earliest_departure_time;
				pSTSNetwork[thread_no].m_arrival_time_ending = p_agent->latest_arrival_time;
				pSTSNetwork[thread_no].m_agent_vector.push_back(a);

				trip_price = pSTSNetwork[thread_no].find_STS_path_for_agents_assigned_for_this_thread(number_of_threads, LR_iteration, optimization_method);

				// we record the time-dependent link visit count from each agent 
				total_price += trip_price;

				int agent_id = g_agent_vector[a].agent_id;

				if (g_pFileDebugLog_LR != NULL)
				{
					fprintf(g_pFileDebugLog_LR, "no. %d: agent_id = %d, trip_price = %0.2f\n", a, agent_id, trip_price);
				}
			}

		}  // end for each agent 

		// step 4: count total resource price 
		float total_resource_price = 0;

		for (int l = 0; l < g_link_vector.size(); l++)
		{
			if (g_link_vector[l].service_type == enum_road_capacity_link_type)
			{
				for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
				{
					total_resource_price += g_link_vector[l].time_dependent_LR_multiplier_vector[t] * g_link_vector[l].time_depedent_capacity_vector[t];
				}
			}
		}


		LR_global_lower_bound = total_price - total_resource_price;
		g_best_lower_bound = max(g_best_lower_bound, LR_global_lower_bound);


	} //End for LR iteration

	cout << "End of LR Optimization Process. " << endl;
	return true;
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
	//g_TrafficAssignment();
	//g_TrafficSimulation();
	////LR
	////// step 2: initialize the resource matrix in upper bound
	////// step 3: Lagrangian optimization for Lower and Upper bound
	g_LR_Optimization(0);
	//g_LR_Optimization_based_on_dynamic_state_array();
	g_OutputFiles();

	//cout << "End of Optimization " << endl;
	//cout << "free memory.." << endl;
	//cout << "done." << endl;
	//delete[] pNetworkForSP;

	g_node_vector.clear();
	g_link_vector.clear();
	g_agent_vector.clear();

	return 1;
}

