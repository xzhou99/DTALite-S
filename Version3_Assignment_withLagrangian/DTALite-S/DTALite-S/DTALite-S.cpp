
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <list> 
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "CSVParser.h"
#include <functional>
#include <stdio.h>   
#include <tchar.h>

#include <windows.h>


#define _MAX_LABEL_COST 99999
#define _MAX_PROCESSOR_SIZE 64
#define _MAX_STATES 7

#define _MAX_NUMBER_OF_TIME_INTERVALS 100

#define _MAX_TAU_SIZE 1440 

TCHAR g_DTASettingFileName[_MAX_PATH] = _T("./DTASettings.txt");


// Linear congruential generator 
#define LCG_a 17364
#define LCG_c 0
#define LCG_M 65521  // it should be 2^32, but we use a small 16-bit number to save memory


template <typename T>
T **Allocate2DDynamicArray(int nRows, int nCols)
{
	T **dynamicArray;

	dynamicArray = new T*[nRows];

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new T[nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

	}

	return dynamicArray;
}

template <typename T>
void Deallocate2DDynamicArray(T** dArray, int nRows)
{
	for (int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete[] dArray;

}
template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}

	for (int x = 0; x < nX; x++)
	{
		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}
		}
	}

	return dynamicArray;

}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;

}


// The one and only application object

/* ***************************************************************************** */
/* Copyright:      Francois Panneton and Pierre L'Ecuyer, University of Montreal */
/*                 Makoto Matsumoto, Hiroshima University                        */
/* Notice:         This code can be used freely for personal, academic,          */
/*                 or non-commercial purposes. For commercial purposes,          */
/*                 please contact P. L'Ecuyer at: lecuyer@iro.UMontreal.ca       */
/* ***************************************************************************** */
#define W 32
#define R 16
#define P 0
#define M1 13
#define M2 9
#define M3 5

#define MAT0POS(t,v) (v^(v>>t))
#define MAT0NEG(t,v) (v^(v<<(-(t))))
#define MAT3NEG(t,v) (v<<(-(t)))
#define MAT4NEG(t,b,v) (v ^ ((v<<(-(t))) & b))

#define V0            STATE[state_i                   ]
#define VM1           STATE[(state_i+M1) & 0x0000000fU]
#define VM2           STATE[(state_i+M2) & 0x0000000fU]
#define VM3           STATE[(state_i+M3) & 0x0000000fU]
#define VRm1          STATE[(state_i+15) & 0x0000000fU]
#define VRm2          STATE[(state_i+14) & 0x0000000fU]
#define newV0         STATE[(state_i+15) & 0x0000000fU]
#define newV1         STATE[state_i                 ]
#define newVRm1       STATE[(state_i+14) & 0x0000000fU]

#define FACT 2.32830643653869628906e-10

static unsigned int STATE[R];
static unsigned int z0, z1, z2;
static unsigned int state_i = 0;

void InitWELLRNG512a(unsigned int *init) {
	int j;
	state_i = 0;
	for (j = 0; j < R; j++)
		STATE[j] = init[j];
}

double WELLRNG512a(void) {
	z0 = VRm1;
	z1 = MAT0NEG(-16, V0) ^ MAT0NEG(-15, VM1);
	z2 = MAT0POS(11, VM2);
	newV1 = z1                  ^ z2;
	newV0 = MAT0NEG(-2, z0) ^ MAT0NEG(-18, z1) ^ MAT3NEG(-28, z2) ^ MAT4NEG(-5, 0xda442d24U, newV1);
	state_i = (state_i + 15) & 0x0000000fU;
	return ((double)STATE[state_i]) * FACT;
}


CWinApp theApp;
using namespace std;

TCHAR g_SettingFileName[_MAX_PATH] = _T("./Settings.txt");

//
//int g_GetPrivateProfileInt(LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename, bool print_out)
//{
//	char lpbuffer[64];
//	int value = def_value;
//	if (GetPrivateProfileString(section, key, "", lpbuffer, sizeof(lpbuffer), filename))
//	{
//		value = atoi(lpbuffer);
//	}
//
//	if (value == def_value)  //  the parameter might not exist
//	{
//		sprintf_s(lpbuffer, "%d", def_value);
//		WritePrivateProfileString(section, key, lpbuffer, filename);
//	}
//
//	if (print_out)
//		cout << "section <" << section << ">: " << key << " = " << value << endl;
//
//	return value;
//}

FILE* g_pFileDebugLog = NULL;

FILE* g_pFileOutputLog = NULL;

FILE* g_pTSViewOutput = NULL;

int g_number_of_threads = 1;
int g_shortest_path_debugging_flag = 0;
int g_number_of_agents;
int g_NumberOfIterations = 1;
int g_TrafficFlowModel = 1;
int g_generate_agent_csv = 1;
int g_generate_link_TDMOE_csv = 0;


int max_number_of_links_per_path = 50;


#define _MAX_ZONE_SIZE 4000
double g_loading_multiplier = 0.66666666;
double g_ODME_adjusment_step_size = 0.01;
int g_max_number_of_agents = 5000000;
double g_OD_loading_multiplier[_MAX_ZONE_SIZE][_MAX_ZONE_SIZE];

int g_number_of_simulation_intervals = 1200;  // 3600 per seconds  
int g_number_of_seconds_per_interval = 6;
int g_number_of_intervals_per_min = 60/ g_number_of_seconds_per_interval;
int g_number_of_simulation_minutes = 100;

int g_Simulation_StartTimeInMin = 9999;
int g_Simulation_EndTimeInMin = 0;
int g_Post_Simulation_DurationInMin = 120;


// 6 seconds per interval
// 3600 -> 6
// 1800 -> 3
// 900 -> 1.5

std::map<int, int> g_link_key_to_seq_no_map;  // hush table, map key to internal link sequence no. 
extern double WELLRNG512a(void);
extern void InitWELLRNG512a(unsigned int *init);
double g_GetRandomRatio()
{
	return WELLRNG512a();
}
// end of random number seeds


typedef struct
{
	int agent_id;
	int agent_type;
	int from_origin_node_id;
	int to_destination_node_id;
	int vehicle_seat_capacity;
	int PCE_factor;
	float departure_time_in_min;
	float travel_time_in_min;
	float arrival_time_in_min;

	int fixed_path_flag;
	int number_of_nodes;
	int path_index;
	//path_node_sequence, path_time_sequence
	
} struct_AgentInfo_Header;


typedef struct
{
	int node_id;      //external node number 
	int zone_id;

} struct_NodeInfo_Header;

typedef struct
{
	int from_node_id;
	int to_node_id;
	int link_type;
	int service_type;
	float length;
	float speed_limit;
	float BPR_alpha_term;
	float BPR_beta_term;
	int number_of_lanes;
	int lane_cap;
	int jam_density;
} struct_LinkInfo_Header;


//mfd
int g_TAU;


std::map<int, int> g_internal_node_seq_no_map;  // hush table, map external node number to internal node sequence no. 


long g_GetLinkSeqNo(int from_node_no, int to_node_no)
{
	if (g_internal_node_seq_no_map.find(from_node_no) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	if (g_internal_node_seq_no_map.find(to_node_no) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	int from_node_seq_no = g_internal_node_seq_no_map[from_node_no];
	int to_node_seq_no = g_internal_node_seq_no_map[to_node_no];

	long link_key = from_node_seq_no * 100000 + to_node_seq_no;

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
		lane_cap = 1000;
		service_type = 0;
		cost = 0;
		BPR_alpha_term = 0.15f;
		BPR_beta_term = 4.0f;
		link_capacity = 1000;
		jam_density = 100;
		free_flow_travel_time_in_min = 1;
		flow_volume = 0;
		number_of_lanes = 1;
		// mfd
		mfd_zone_id = 0;

		m_LinkOutFlowCapacity = NULL;
		m_LinkInFlowCapacity = NULL;
		m_LinkCumulativePerMinArrival = NULL;
		m_LinkCumulativePerMinDeparture = NULL;
		m_LinkCumulativePerMinVirtualDelay = NULL;
		m_LinkTravelTime = NULL;

		m_LinkObsFlow = NULL; 
		m_LinkObsDensity = NULL;
		m_LinkObsTravelTime = NULL;
		m_LinkObsFlowDeviation = NULL;
		m_LinkObsDensityDeviation = NULL;
		m_LinkObsMarginal = NULL;
		speed_limit = 10;

	}

	~CLink()
	{
		DeallocateMemory();
	}
	
	std::list<int>  m_waiting_traveler_queue;


	int state_travel_time_vector[_MAX_STATES];

	int** state_dependent_travel_time_matrix;
	float* time_dependent_flow_volume_vector;
	float** state_dependent_time_dependent_LR_multiplier_matrix;

	// (i,j), from t at mode m--> 
	// method 1: determined by train schedule from input_agent.csv, path_schedule_time_sequence
	// method 2: on pick up links, from given time t, find the next available k trains at time t', TT= t'-t
	// method 3: on freeway links, at given time t, use state_travel_time_vector from input_link.csv to generate t'
	// method 4: on waiting links: departure from home link and to final destination links, provide a feasible range of waiting time at the same mode of 0
	
	void Setup_State_Dependent_Data_Matrix()
	{

		state_dependent_travel_time_matrix = Allocate2DDynamicArray<int>(g_number_of_simulation_minutes, _MAX_STATES);
			
		state_dependent_time_dependent_LR_multiplier_matrix = Allocate2DDynamicArray<float>(g_number_of_simulation_minutes, _MAX_STATES);

		time_dependent_flow_volume_vector = new float[g_number_of_simulation_minutes];

		for (int t = 0; t < g_number_of_simulation_minutes; t++)
		{
			time_dependent_flow_volume_vector[t] = 0;

			for (int s = 0; s < _MAX_STATES; s++)
			{
				state_dependent_travel_time_matrix[t][s] = 0;
				state_dependent_time_dependent_LR_multiplier_matrix[t][s] = 0;

			}
		}
	}

	void ResetTDFlowVolumeVector()
	{
		for (int t = 0; t < g_number_of_simulation_minutes; t++)
		{
			time_dependent_flow_volume_vector[t] = 0;
		}
	}

	// all alocated as relative time
	int* m_LinkOutFlowCapacity;
	int* m_LinkInFlowCapacity;
	int* m_LinkCumulativePerMinArrival;
	int* m_LinkCumulativePerMinDeparture;
	int* m_LinkCumulativePerMinVirtualDelay;

	int m_CumulativeArrivalCount;
	int m_CumulativeDepartureCount;
	int m_CumulativeVirtualDelayCount;


	int m_CumulativeArrivalCountPerMinPerProcessor[_MAX_PROCESSOR_SIZE];
	int m_CumulativeDepartureCountPerMinPerProcessor[_MAX_PROCESSOR_SIZE];

	void ResetPerMinADCounts(int number_of_threads)
	{
		for (int p = 0; p < number_of_threads; p++)
		{
			m_CumulativeArrivalCountPerMinPerProcessor[p] = 0;
			m_CumulativeDepartureCountPerMinPerProcessor[p] = 0;
		}
	}

	void TallyPerMinADCounts(int number_of_threads, int time_in_min)
	{
		for (int p = 0; p < number_of_threads; p++)  // scan through AD counts across all processors
		{
			m_CumulativeArrivalCount+=m_CumulativeArrivalCountPerMinPerProcessor[p];
			m_CumulativeDepartureCount+=m_CumulativeDepartureCountPerMinPerProcessor[p];
		}
		m_LinkCumulativePerMinArrival[time_in_min] = m_CumulativeArrivalCount;
		m_LinkCumulativePerMinDeparture[time_in_min] = m_CumulativeDepartureCount;
	}

	float* m_LinkObsFlow;
	float* m_LinkObsDensity;   //
	float* m_LinkObsTravelTime;  // in min
	float* m_LinkObsFlowDeviation;
	float* m_LinkObsDensityDeviation;   //
	float* m_LinkObsMarginal;

	float* m_LinkTravelTime;

	float GetLinkCapacityPerSimulationInterval(float link_capacity_per_hour)
	{
		return link_capacity*g_number_of_seconds_per_interval / 3600.0;  // 3600.0 sec per hour
	}

	void AllocateMemory()
	{
		m_LinkOutFlowCapacity = new int[g_number_of_simulation_intervals];
		m_LinkInFlowCapacity = new int[g_number_of_simulation_intervals];
		m_LinkCumulativePerMinArrival = new int[g_number_of_simulation_minutes];
		m_LinkCumulativePerMinDeparture = new int[g_number_of_simulation_minutes];
		m_LinkCumulativePerMinVirtualDelay = new int[g_number_of_simulation_minutes];

		m_LinkTravelTime = new float[g_number_of_simulation_minutes];

		m_LinkObsFlow = new float[g_number_of_simulation_minutes];
		m_LinkObsDensity = new float[g_number_of_simulation_minutes];
		m_LinkObsTravelTime = new float[g_number_of_simulation_minutes];
		m_LinkObsFlowDeviation = new float[g_number_of_simulation_minutes];
		m_LinkObsDensityDeviation = new float[g_number_of_simulation_minutes];
		m_LinkObsMarginal = new float[g_number_of_simulation_minutes];

	

		for (int t = 0; t < g_number_of_simulation_intervals; t++)
		{

			double float_value = GetLinkCapacityPerSimulationInterval(link_capacity);
			int int_value = (int)(float_value); // round off
			double redidual = float_value - int_value;

			int time_dependent_capacity_round_to_nearest_integer;

			double random_ratio = g_GetRandomRatio();

			// e.g. redidual = 0.7
			// random_ratio = 0.3

			if (redidual > random_ratio)  // e.g. random_ratio is bewtween 0 and 0.7 
			{
				time_dependent_capacity_round_to_nearest_integer = int_value + 1;
			}
			else
			{
				time_dependent_capacity_round_to_nearest_integer = int_value;
			}

			m_LinkOutFlowCapacity[t] = time_dependent_capacity_round_to_nearest_integer;
		}
			
		for (int t = 0; t < g_number_of_simulation_minutes; t++)
		{
			m_LinkCumulativePerMinArrival[t] = 0;
			m_LinkCumulativePerMinDeparture[t] = 0;
			m_LinkCumulativePerMinVirtualDelay[t] = 0;

			m_LinkObsDensity[t] = -1;
			m_LinkObsTravelTime[t] = -1;
			m_LinkObsFlowDeviation[t] = -1;
			m_LinkObsDensityDeviation[t] = -1;
			m_LinkObsMarginal[t] = 0;
		}

		free_flow_travel_time_in_simu_interval = max(1,int(free_flow_travel_time_in_min*60.0 / g_number_of_seconds_per_interval + 0.5));
	}

	void ResetMOE()
	{
		m_CumulativeArrivalCount = 0;
		m_CumulativeDepartureCount = 0;
		m_CumulativeVirtualDelayCount = 0;


	}

	void DeallocateMemory()
	{
		////if(m_LinkOutFlowCapacity != NULL) delete m_LinkOutFlowCapacity;
		////if (m_LinkInFlowCapacity != NULL) delete m_LinkInFlowCapacity;
		////if (m_LinkCumulativePerMinArrival != NULL) delete m_LinkCumulativePerMinArrival;
		////if (m_LinkCumulativePerMinDeparture != NULL) delete m_LinkCumulativePerMinDeparture;
		////if (m_LinkTravelTime != NULL) delete m_LinkTravelTime;

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
	int from_node_id;   // external node numbers, 
	int to_node_id;

	int link_seq_no;
	int from_node_seq_no;  // starting from 0, sequential numbers 
	int to_node_seq_no;
	float cost;
	float free_flow_travel_time_in_min;
	int free_flow_travel_time_in_simu_interval;
	float speed_limit;
	int number_of_lanes;

	int lane_capacity;
	int lane_cap;
	int type;

	int service_type; // 0: moving, -1: drop off, +1, pick up
	float link_capacity;  // per hour
	float link_capacity_per_min;  // per hour

	float jam_density;
	float flow_volume;
	float travel_time;
	float BPR_alpha_term;
	float BPR_beta_term;
	float length; 
	// mfd
	int mfd_zone_id;

	void CalculateBRPFunction()
	{
		travel_time = free_flow_travel_time_in_min*(1 + BPR_alpha_term*pow(flow_volume / max(0.00001, link_capacity), BPR_beta_term));
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
		if (m_LinkCumulativePerMinArrival == NULL)
			return 0;

		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin - 1)
			return m_LinkCumulativePerMinArrival[(time_in_min + 1) ] - m_LinkCumulativePerMinArrival[time_in_min];
		else
			return 0;

	}

	float get_link_out_flow_per_min(int time_in_min)
	{
		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin - 1)
			return m_LinkCumulativePerMinDeparture[(time_in_min + 1) ] - m_LinkCumulativePerMinDeparture[time_in_min ];
		else
			return 0;

	}

	float get_number_of_vehicles(int time_in_min)
	{
		if (m_LinkCumulativePerMinArrival == NULL)
			return 0;

		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin )
			return m_LinkCumulativePerMinArrival[(time_in_min ) ] - m_LinkCumulativePerMinDeparture[time_in_min ];
		else
			return 0;

	}


	float get_avg_delay_in_min(int time_in_min, int time_duration)
	{
		if (m_LinkCumulativePerMinVirtualDelay == NULL)
			return 0;

		if (time_in_min + time_duration < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin)
		{
			float total_delay_in_min = (m_LinkCumulativePerMinVirtualDelay[(time_in_min + time_duration) * 60 / g_number_of_seconds_per_interval] - m_LinkCumulativePerMinVirtualDelay[time_in_min * 60 / g_number_of_seconds_per_interval])*g_number_of_seconds_per_interval/60.0;
			int avg_number_of_vehicle = (get_number_of_vehicles(time_in_min) + get_number_of_vehicles(time_in_min + time_duration)) / 2;
			return total_delay_in_min / max(1, avg_number_of_vehicle);
		}
		else
			return 0;
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
		m_SeqNo4OriginNode = -1;
	}

	int accessible_node_count;

	int node_seq_no;  // sequence number 
	int node_id;      //external node number 
	int zone_id;
	double x;
	double y;

	bool bOriginNode_ForAgents;
	int m_SeqNo4OriginNode;

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
		agent_type = 0;  //0: pax vehicle 1: travler 2: scheduled transportation vehicle
		m_bMoveable = 1;
		fixed_path_flag = 0;
		vehicle_seat_capacity = 1;
		m_bGenereated = false;
		PCE_factor = 1.0;
		travel_time_in_min = 0;
		m_Veh_LinkArrivalTime = NULL;
		m_Veh_LinkDepartureTime = NULL;
		m_bCompleteTrip = false;
		path_index = -1;
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

	int fixed_path_flag;
	int agent_id;

	int agent_vector_seq_no;
	int agent_type;
	int m_bMoveable;
	int origin_node_id;
	int destination_node_id;
	int path_index;

	int origin_zone_seq_no;
	int destination_zone_seq_no;

	float departure_time_in_min;
	int departure_time_in_simulation_interval;

	float PCE_factor;  // passenger car equivalent : bus = 3
	float travel_time_in_min; 
	std::vector<int> path_link_seq_no_vector;
	std::vector<int> path_timestamp_vector;

	int m_path_link_seq_no_vector_size;

	std::vector<int> path_node_id_vector;
	std::vector<int> path_schedule_time_vector;
	std::vector<float> tsview_timestamp_vector;

	int m_current_link_seq_no;
	int* m_Veh_LinkArrivalTime;
	int* m_Veh_LinkDepartureTime;

	int vehicle_seat_capacity;

	std::list<int>  m_PassengerList;  

	void Pickup(int p)
	{
		if(m_PassengerList.size() < vehicle_seat_capacity)
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

	void AllocateMemory(int link_memory_flag = 0)
	{
		if (m_Veh_LinkArrivalTime == NULL)
		{
			m_current_link_seq_no = 0;
			m_Veh_LinkArrivalTime = new int[path_link_seq_no_vector.size()];
			m_Veh_LinkDepartureTime = new int[path_link_seq_no_vector.size()];
		}
		for(int i = 0; i < path_link_seq_no_vector.size(); i++)
		{ 
			m_Veh_LinkArrivalTime[i] = -1;
			m_Veh_LinkDepartureTime[i] = -1;

		}

		m_path_link_seq_no_vector_size = path_link_seq_no_vector.size();
		departure_time_in_simulation_interval = int(departure_time_in_min*60.0 / g_number_of_seconds_per_interval + 0.5);  // covert departure time in min to an integer value of simulation time intervals

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

class CAgentElement
{
public:
	CAgentElement()
	{
		bActive = true;
	}


	int agent_no;
	bool bActive;
};

vector<CAgent> g_agent_vector;
std::map<int, int> g_map_agent_id_to_agent_vector_seq_no;

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


void g_WriteNetworkBinFile()
{
	// write output agent bin file
	FILE* st_struct = NULL;
	int err = 0;

	// node block
	err = fopen_s(&st_struct, "output_node.bin", "wb");
	if (err > 0)
	{
		cout << "The file 'output_node.bin' was not opened\n";
		g_ProgramStop();
	}

	struct_NodeInfo_Header header_node;
	cout << "output output_node.bin'" << endl;

	for (int i = 0; i < g_node_vector.size(); i++)
	{

		header_node.node_id = g_node_vector[i].node_id;
		header_node.zone_id = g_node_vector[i].zone_id;

		fwrite(&header_node, sizeof(struct_NodeInfo_Header), 1, st_struct);

	}
	fclose(st_struct);


	//


	// link block
	err = fopen_s(&st_struct, "output_link.bin", "wb");
	if (err > 0)
	{
		cout << "The file 'output_link.bin' was not opened\n";
		g_ProgramStop();
	}
	cout << "output output_link.bin'" << endl;

	struct_LinkInfo_Header header_link;

	for (int l = 0; l < g_link_vector.size(); l++)
	{
		header_link.from_node_id = g_link_vector[l].from_node_id;
		header_link.to_node_id = g_link_vector[l].to_node_id;
		header_link.link_type = g_link_vector[l].type;
		header_link.service_type = g_link_vector[l].service_type;
		header_link.length = g_link_vector[l].length;
		header_link.speed_limit = g_link_vector[l].speed_limit;
		header_link.BPR_alpha_term = g_link_vector[l].BPR_alpha_term;
		header_link.BPR_beta_term = g_link_vector[l].BPR_beta_term;
		header_link.number_of_lanes = g_link_vector[l].number_of_lanes;
		header_link.lane_cap = g_link_vector[l].lane_cap;
		header_link.jam_density = g_link_vector[l].jam_density;

		fwrite(&header_link, sizeof(struct_LinkInfo_Header), 1, st_struct);
	}
	fclose(st_struct);
	// agent block
	struct_AgentInfo_Header header;

	err = fopen_s(&st_struct, "output_agent.bin", "wb");
	if (err > 0)
	{
		cout << "The file 'output_agent.bin' was not opened\n";
		g_ProgramStop();
	}

	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		CAgent* p_agent = &(g_agent_vector[a]);
		header.agent_id = p_agent->agent_id;
		header.agent_type = p_agent->origin_node_id;
		header.from_origin_node_id = p_agent->origin_node_id;
		header.to_destination_node_id = p_agent->destination_node_id;
		header.vehicle_seat_capacity = p_agent->vehicle_seat_capacity;
		header.PCE_factor = p_agent->PCE_factor;
		header.departure_time_in_min = p_agent->departure_time_in_min;
		header.travel_time_in_min = p_agent->travel_time_in_min;
		header.fixed_path_flag = p_agent->fixed_path_flag;
		header.number_of_nodes = p_agent->path_node_id_vector.size();
		header.path_index = -1;
		fwrite(&header, sizeof(struct_AgentInfo_Header), 1, st_struct);

	}
	fclose(st_struct);


}

void g_ReadInputScenarioFile()
{
	int RandomSeed = 1;

	CCSVParser parser_scenario;

	if (parser_scenario.OpenCSVFile("input_scenario_settings.csv", true))
	{
		while (parser_scenario.ReadRecord())
		{

			parser_scenario.GetValueByFieldName("random_seed", RandomSeed);


			// end of random number seeds

			if (parser_scenario.GetValueByFieldName("number_of_assignment_days", g_NumberOfIterations) == false)
			{
				if (parser_scenario.GetValueByFieldName("number_of_iterations", g_NumberOfIterations) == false)
				{
					cout << "Field number_of_iterations cannot be found in file input_scenario_settings.csv. Please check." << endl;
					g_ProgramStop();
				}
			}

	
			if (parser_scenario.GetValueByFieldName("traffic_flow_model", g_TrafficFlowModel) == false)
			{
				cout << "Field traffic_flow_model cannot be found in file input_scenario_settings.csv. Please check." << endl;
				g_ProgramStop();
			}

			int UEAssignmentMethod = 0;

			if (parser_scenario.GetValueByFieldName("traffic_analysis_method", UEAssignmentMethod) == false)
			{
			}

			parser_scenario.GetValueByFieldName("generate_agent_csv", g_generate_agent_csv);
			parser_scenario.GetValueByFieldName("generate_link_TDMOE_csv", g_generate_link_TDMOE_csv);

			

		}
	}

	RandomSeed = max(1, RandomSeed);
	unsigned int state[16];

	for (int k = 0; k < 16; ++k)
	{
		state[k] = k + RandomSeed;
	}

	InitWELLRNG512a(state);
}
int g_ReadInputBinFile()
{

	int internal_node_seq_no = 0;
	// step 1: read node file 


	FILE* st = NULL;
	fopen_s(&st, "input_node.bin", "rb");
	if (st != NULL)
	{
		struct_NodeInfo_Header header;

		int count = 0;
		while (!feof(st))
		{

			size_t result = fread(&header, sizeof(struct_NodeInfo_Header), 1, st);
			if (header.node_id <= 0)
				break; 
			
			if (g_internal_node_seq_no_map.find(header.node_id) != g_internal_node_seq_no_map.end())
			{
				continue; //has been defined
			}
			g_internal_node_seq_no_map[header.node_id] = internal_node_seq_no;


			CNode node;  // create a node object 

			node.node_id = header.node_id;
			node.node_seq_no = internal_node_seq_no;
			node.zone_id = header.zone_id;

			internal_node_seq_no++;

			g_node_vector.push_back(node);  // push it to the global node vector

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 == 0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}

		cout << "number of nodes = " << g_number_of_nodes << endl;

		fprintf(g_pFileOutputLog, "number of nodes =,%d\n", g_number_of_nodes);
		fclose(st);
	}

	// step 2: read link file 

	fopen_s(&st, "input_link.bin", "rb");
	if (st != NULL)
	{
		struct_LinkInfo_Header header;

		int count = 0;
		while (!feof(st))
		{

			size_t result = fread(&header, sizeof(struct_LinkInfo_Header), 1, st);
			if (header.from_node_id <= 0)
				break;
			CLink link; 
			
			link.from_node_id = header.from_node_id;
			link.to_node_id = header.to_node_id;

			int internal_from_node_seq_no = g_internal_node_seq_no_map[link.from_node_id];  // map external node number to internal node seq no. 
			int internal_to_node_seq_no = g_internal_node_seq_no_map[link.to_node_id];


			link.from_node_seq_no = internal_from_node_seq_no;
			link.to_node_seq_no = internal_to_node_seq_no;
			link.link_seq_no = g_number_of_links;
			link.to_node_seq_no = internal_to_node_seq_no;

			link.type = header.link_type;
			link.service_type = header.service_type;

			link.length = header.service_type;;
			link.speed_limit = header.speed_limit;
			link.BPR_alpha_term = header.BPR_alpha_term;
			link.BPR_beta_term = header.BPR_beta_term;
			link.number_of_lanes = header.number_of_lanes;
			link.lane_cap = header.lane_cap;
			link.jam_density = header.jam_density;

			link.link_capacity = link.lane_cap* header.number_of_lanes;
			link.link_capacity_per_min = link.link_capacity / 60; // convert per hour capacity to per min capacity 

			link.free_flow_travel_time_in_min = link.length / link.speed_limit * 60;
			link.length = link.length;
			link.cost = max(0.1, link.length / link.speed_limit * 60); // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate delay 


			g_node_vector[internal_from_node_seq_no].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link

			long link_key = internal_from_node_seq_no * 100000 + internal_to_node_seq_no;

			g_link_key_to_seq_no_map[link_key] = link.link_seq_no;
			g_link_vector.push_back(link);

			g_number_of_links++;

			if (g_number_of_links % 1000 == 0)
				cout << "reading " << g_number_of_links << " links.. " << endl;
		}
		cout << "number of links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);

		fclose(st);
	}





	fopen_s(&st, "input_agent.bin", "rb");
	if (st != NULL)
	{
		struct_AgentInfo_Header header;

		int count = 0;
		while (!feof(st))
		{

			size_t result = fread(&header, sizeof(struct_AgentInfo_Header), 1, st);

			if (header.agent_id < 0)
				break;


			if (g_number_of_agents >= g_max_number_of_agents)
				break;


			CAgent agent;  // create an agent object 

			agent.agent_id = header.agent_id;
			agent.agent_type = header.agent_type,
				agent.origin_node_id = header.from_origin_node_id;
			agent.destination_node_id = header.to_destination_node_id;
			agent.vehicle_seat_capacity = header.vehicle_seat_capacity;
			agent.PCE_factor = header.PCE_factor;
			agent.departure_time_in_min = header.departure_time_in_min;
			agent.fixed_path_flag = header.fixed_path_flag;

			agent.m_RandomSeed = agent.agent_id;
			if (g_internal_node_seq_no_map.find(agent.origin_node_id) == g_internal_node_seq_no_map.end() || g_internal_node_seq_no_map.find(agent.destination_node_id) == g_internal_node_seq_no_map.end())
				continue;

			if (agent.departure_time_in_min < g_Simulation_StartTimeInMin)
				g_Simulation_StartTimeInMin = agent.departure_time_in_min;

			if (agent.departure_time_in_min > g_Simulation_EndTimeInMin)
				g_Simulation_EndTimeInMin = agent.departure_time_in_min;


			g_agent_vector.push_back(agent);
			g_number_of_agents++;
			if (g_number_of_agents % 1000 == 0)
				cout << "reading = " << g_number_of_agents / 1000 << " k agents..." << endl;

		}


		fclose(st);
	}

	if (g_number_of_nodes > 0 && g_number_of_links > 0 && g_number_of_agents > 0)
		return 1;
	else 
		return 0;
}

void g_ReadInputData()
{

	g_ReadInputScenarioFile();

	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0

	if (g_ReadInputBinFile() >= 1)
		return;
	


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
			CLink link;  // create a link object 

			if (parser_link.GetValueByFieldName("from_node_id", link.from_node_id) == false)
				continue;
			if (parser_link.GetValueByFieldName("to_node_id", link.to_node_id) == false)
				continue;

				// add the to node id into the outbound (adjacent) node list

			int internal_from_node_seq_no = g_internal_node_seq_no_map[link.from_node_id];  // map external node number to internal node seq no. 
			int internal_to_node_seq_no = g_internal_node_seq_no_map[link.to_node_id];
			

			link.from_node_seq_no = internal_from_node_seq_no;
			link.to_node_seq_no = internal_to_node_seq_no;
			link.link_seq_no = g_number_of_links;
			link.to_node_seq_no = internal_to_node_seq_no;
	
			parser_link.GetValueByFieldName("link_type", link.type);
			parser_link.GetValueByFieldName("service_type", link.service_type, false);
			
			float length =1; // km or mile

			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("speed_limit", link.speed_limit);

			parser_link.GetValueByFieldName("BPR_alpha_term", link.BPR_alpha_term);
			parser_link.GetValueByFieldName("BPR_beta_term", link.BPR_beta_term);
			int number_of_lanes = 1;

			parser_link.GetValueByFieldName("number_of_lanes", link.number_of_lanes);
			parser_link.GetValueByFieldName("lane_cap", link.lane_cap);
			parser_link.GetValueByFieldName("jam_density", link.jam_density);
			
		
			link.link_capacity = link.lane_cap* number_of_lanes;

			link.free_flow_travel_time_in_min = length / link.speed_limit * 60;


			parser_link.GetValueByFieldName("travel_time_s0", link.state_travel_time_vector[0]);
			parser_link.GetValueByFieldName("travel_time_s1", link.state_travel_time_vector[1]);
			parser_link.GetValueByFieldName("travel_time_s2", link.state_travel_time_vector[2]);
			parser_link.GetValueByFieldName("travel_time_s3", link.state_travel_time_vector[3]);
			parser_link.GetValueByFieldName("travel_time_s4", link.state_travel_time_vector[4]);
			parser_link.GetValueByFieldName("travel_time_s5", link.state_travel_time_vector[5]);
			parser_link.GetValueByFieldName("travel_time_s6", link.state_travel_time_vector[6]);


			link.length = length;
			link.cost = length / link.speed_limit * 60; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate delay 


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

		// set NFD zone jam 


		g_number_of_agents = 0;


		if(g_number_of_agents ==0)
		{
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

				if (g_number_of_agents >= g_max_number_of_agents)
					break;

				parser_agent.GetValueByFieldName("agent_type", agent.agent_type);
				
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

				if(agent.path_node_id_vector.size() >=2)
				{
				for (int n = 0; n < agent.path_node_id_vector.size() - 1; n++)
				{
					int link_seq_no = g_GetLinkSeqNo(agent.path_node_id_vector[n], agent.path_node_id_vector[n+1]);
					
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

				g_agent_vector.push_back(agent);
				g_number_of_agents++;
				if(g_number_of_agents%1000==0)
					cout << "reading = " << g_number_of_agents/1000 << " k agents..." << endl;

			}
		}
		parser_agent.CloseCSVFile();
		}
		cout << "number of agents = " << g_agent_vector.size() << endl;
		g_WriteNetworkBinFile();

		cout << " Sort agents... " << endl;
		std::sort(g_agent_vector.begin(), g_agent_vector.end());

		// simulation
		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			g_agent_vector[a].agent_vector_seq_no = a;

			g_map_agent_id_to_agent_vector_seq_no[g_agent_vector[a].agent_id] = a;
		}
		cout << " Sorting ends. ..." << endl;		
		// 120 min at the end fo simulation
		g_number_of_simulation_intervals = (g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin) * 60 / g_number_of_seconds_per_interval;
		g_number_of_simulation_minutes = (g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin);

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


	int m_private_origin_seq_no;
	std::vector<int>  m_agent_vector; // assigned agents for computing 
	std::vector<int>  m_node_vector; // assigned nodes for computing 
	std::vector<CNode2NodeAccessibility>  m_node2node_accessibility_vector;
	
	
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
			
		// perform shortest path calculation
	
		if (m_private_origin_seq_no < 0)
			return;

		int return_value = optimal_label_correcting(m_private_origin_seq_no,-1, 0);   // step 1: we first perfor label correcting for all agents belonging to this zone

	
		// step 2: loop throug all agents 
		for (int i = 0; i < m_agent_vector.size(); i++)
		{

			CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

			if (p_agent->fixed_path_flag == 1)
				continue;
			//step 3: use MSA rule to determine which agent will be assigned to new path 

				// ratio_for_updating_path_at_this_iteration = 1 / (assignment_iteration_no + 1);  //1/1, 1/2, 1/3, 1/4

				int residual = i % (assignment_iteration_no + 1); // MSA rule

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


				if (return_value == -1)
			{
				fprintf(g_pFileDebugLog, "agent %d with can not find destination node,\n", i);
				continue;
			}

			int current_node_seq_no;
			int current_link_seq_no;

			//step 4: back trace from the destination node to find the shortest path from shortest path tree 
			current_node_seq_no = g_internal_node_seq_no_map[p_agent->destination_node_id];
			p_agent->travel_time_in_min = m_node_label_cost [ g_internal_node_seq_no_map[p_agent->destination_node_id]];

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
				
				     p_agent->path_node_id_vector.push_back(g_node_vector[current_node_seq_no].node_id);

				}


				current_node_seq_no = m_node_predecessor[current_node_seq_no];


			}

					if(p_agent->path_link_seq_no_vector.size() >=max_number_of_links_per_path )
					{ 
		#pragma omp critical
				{
					max_number_of_links_per_path = max(p_agent->path_link_seq_no_vector.size(),max_number_of_links_per_path);

				}
			}
			
			if (pFileAgentPathLog != NULL)
				fprintf(pFileAgentPathLog, "\n");


			}

		// step 5: 	scan the shortest path to compute the link volume, 
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

class CSTS_State  //class for space time states
{
public:
	float m_speed;


	std::vector<int> m_outgoing_state_index_vector;
	std::vector<int> m_outgoing_state_transition_cost_vector;


};

std::vector<CSTS_State> g_STSStateVector;

void g_add_state_transition(int from_state, int to_state, float TransitionCost)
{
	g_STSStateVector[from_state].m_outgoing_state_index_vector.push_back(to_state);  // link my own state index to the parent state
	g_STSStateVector[from_state].m_outgoing_state_transition_cost_vector.push_back(TransitionCost);  // link my own state index to the parent state
}

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
	int*** m_node_predecessor;
	int*** m_time_predecessor;
	int*** m_state_predecessor;



	STSNetwork()
	{
		m_origin_node = -1;
		m_label_cost = NULL;
		m_node_predecessor = NULL;
		m_time_predecessor = NULL;
		m_state_predecessor = NULL;

	}


	void AllocateSTSMemory(int number_of_nodes, int number_of_time_intervals, int number_of_states)
	{
		m_number_of_nodes = number_of_nodes;
		m_number_of_time_intervals = number_of_time_intervals;
		m_number_of_states = number_of_states;

		m_label_cost = Allocate3DDynamicArray<float>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_node_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_time_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_state_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
	
	}

	~STSNetwork()
	{
		Deallocate3DDynamicArray<float>(m_label_cost, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_node_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_time_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_state_predecessor, m_number_of_nodes, m_number_of_time_intervals);
	}


	//parallel computing version
	float optimal_STS_dynamic_programming(int departure_time_beginning,
		int arrival_time_ending)
	{
		if (m_origin_node < 0)
			return -1;

		float total_cost = _MAX_LABEL_COST;
		if (g_node_vector[m_origin_node].m_outgoing_node_vector.size() == 0)
		{
			return _MAX_LABEL_COST;
		}

		if (arrival_time_ending >= m_number_of_time_intervals-1)
		{
			return _MAX_LABEL_COST;
		}

		// step 1: Initialization for all nodes
		for (int i = 0; i < m_number_of_nodes; i++) //Initialization for all nodes
		{
			for (int t = departure_time_beginning; t <= arrival_time_ending; t++)
			{

				for (int w = 0; w < m_number_of_states; w++)
				{

					m_label_cost[i][t][w] = _MAX_LABEL_COST;
					m_node_predecessor[i][t][w] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
					m_time_predecessor[i][t][w] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
					m_state_predecessor[i][t][w] = -1;
				}
			}
		}

		//step 2: Initialization for origin node at the preferred departure time, at departure time

		int w0 = 0;  // start fro empty

		m_label_cost[m_origin_node][departure_time_beginning][w0] = 0;

		// step 3: //dynamic programming , absoluate time
		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
		{
			for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
			{
				int from_node = g_link_vector[link].from_node_seq_no;
				int to_node = g_link_vector[link].to_node_seq_no;

				//// if the total travel time from origin to node i then back to destination is greater than the time window, then skip this node/link to scan
				//float travel_time_3_points = g_vehicle_origin_based_node_travel_time[vehicle_id][from_node] + g_vehicle_destination_based_node_travel_time[vehicle_id][from_node];
				//int time_window_length = g_vehicle_arrival_time_ending[vehicle_id] - g_vehicle_departure_time_ending[vehicle_id];

				//// if the total travel time from origin to node i then back to destination is greater than the time window, then skip this node/link to scan
				//if (travel_time_3_points >= time_window_length)
				//	continue;  //skip
				for (int w1 = 0; w1 < m_number_of_states; w1++)
				{

	
					if (m_label_cost[from_node][t][w1] < _MAX_LABEL_COST - 1)  // for feasible time-space point only
					{


						for (int w2_index = 0; w2_index < g_STSStateVector[w1].m_outgoing_state_index_vector.size(); w2_index++)
						{
							int w2 = g_STSStateVector[w1].m_outgoing_state_index_vector[w2_index];
							int travel_time = g_link_vector[link].state_dependent_travel_time_matrix[t][w1];
   						 	int new_to_node_arrival_time = min(t + travel_time, m_number_of_time_intervals - 1);
							
							float to_node_cost = 0;

							float temporary_label_cost = m_label_cost[from_node][t][w1] + 
								g_STSStateVector[w1].m_outgoing_state_transition_cost_vector[w2_index]+
								g_link_vector[link].state_dependent_time_dependent_LR_multiplier_matrix[t][w1]
								;

							if (temporary_label_cost < m_label_cost[to_node][new_to_node_arrival_time][w2]) // we only compare cost at the downstream node ToID at the new arrival time t
							{


								// update cost label and node/time predecessor

								m_label_cost[to_node][new_to_node_arrival_time][w2] = temporary_label_cost;
								m_node_predecessor[to_node][new_to_node_arrival_time][w2] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								m_time_predecessor[to_node][new_to_node_arrival_time][w2] = t;  // pointer to previous TIME INDEX from the current label at current node and time
								m_state_predecessor[to_node][new_to_node_arrival_time][w2] = w1;
							}
														
						}
					}  // feasible vertex label cost
				}  // for all states

			} // for all link
		} // for all time t


		return total_cost;

	}


	void find_STS_path_for_agents_assigned_for_this_thread(int number_of_threads, int assignment_iteration_no)
	{

		int reversed_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		int reversed_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		int reversed_path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		float reversed_path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];

		int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		int path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		int path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		float path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];


		// perform shortest path calculation
		// should be determiend from external input

		// perform one to all STS shortest path 
		int return_value = optimal_STS_dynamic_programming(m_departure_time_beginning, m_arrival_time_ending);   // step 1: we first perfor label correcting for all agents belonging to this zone
																					   // step 2: loop throug all agents 
		for (int i = 0; i < m_agent_vector.size(); i++)
		{

			CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

			if (p_agent->fixed_path_flag == 1)
				continue;
			//step 3: use MSA rule to determine which agent will be assigned to new path 

			// ratio_for_updating_path_at_this_iteration = 1 / (assignment_iteration_no + 1);  //1/1, 1/2, 1/3, 1/4

			int residual = i % (assignment_iteration_no + 1); // MSA rule

			if (residual != 0)  // no need to compute a new path at this iteration
			{
				continue; // that is, it will reuse the path from the previous iteration, stored at p_agent->path_link_seq_no_vector.
			}
			else
			{
				// move to the next line for finding the shortest path 
			}

			p_agent->path_link_seq_no_vector.clear();  // reset;
			p_agent->path_timestamp_vector.clear();
			p_agent->path_node_id_vector.clear();  // reset;


			if (return_value == -1)
			{
				fprintf(g_pFileDebugLog, "agent %d with can not find destination node,\n", i);
				continue;
			}

			int current_node_seq_no;
			int current_link_seq_no;


			//step 4: back trace from the destination node to find the shortest path from shortest path tree 
			int destination_node = p_agent->destination_node_id;

			float total_cost = _MAX_LABEL_COST;

			int min_cost_time_index = m_arrival_time_ending;
			//to do: find min_cost_time_index based on agent's final or preferred arrival time window

			int w = 0;
			total_cost = m_label_cost[destination_node][min_cost_time_index][w];

			// step 2: backtrack to the origin (based on node and time predecessors)
			int	node_size = 0;
			reversed_path_node_sequence[node_size] = destination_node;//record the first node backward, destination node
			reversed_path_time_sequence[node_size] = min_cost_time_index;
			reversed_path_state_sequence[node_size] = w;
			reversed_path_cost_sequence[node_size] = m_label_cost[destination_node][min_cost_time_index][w];


			node_size++;

			int pred_node = m_node_predecessor[destination_node][min_cost_time_index][w];
			int pred_time = m_time_predecessor[destination_node][min_cost_time_index][w];
			int pred_state = m_state_predecessor[destination_node][min_cost_time_index][w];

			while (pred_node != -1 && node_size < _MAX_NUMBER_OF_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
			{
				reversed_path_node_sequence[node_size] = pred_node;
				reversed_path_time_sequence[node_size] = pred_time;
				reversed_path_state_sequence[node_size] = pred_state;
				reversed_path_cost_sequence[node_size] = m_label_cost[pred_node][pred_time][pred_state];

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

			for (int n = 0; n < node_size; n++)
			{
				path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
				path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];
				path_state_sequence[n] = reversed_path_state_sequence[node_size - n - 1];
				path_cost_sequence[n] = reversed_path_cost_sequence[node_size - n - 1];
			}

			for (int i = 0; i < node_size - 1; i++)  // for each link, 
			{

				int link_no = g_GetLinkSeqNo(path_node_sequence[i], path_node_sequence[i + 1]);
				path_link_sequence[i] = link_no;
				p_agent->path_link_seq_no_vector.push_back(link_no);
				p_agent->path_timestamp_vector.push_back(path_time_sequence[i]);

			}

			float  travel_time_return_value = path_time_sequence[node_size - 1] - path_time_sequence[0];

			int path_number_of_nodes = node_size;
		}

		// step 5: 	scan the shortest path to compute the time_dependent link volume, 

#pragma omp critical
		{
			for (int i = 0; i < m_agent_vector.size(); i++)
			{

				CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

				for (int l = 0; l < p_agent->path_link_seq_no_vector.size(); l++)  // for each link in the path of this agent
				{
					int link_seq_no = p_agent->path_link_seq_no_vector[l];
					int t = p_agent->path_timestamp_vector[l];
					g_link_vector[link_seq_no].time_dependent_flow_volume_vector[t] += 1;

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

void g_UpdatePassengerStateInVehicle(CAgent* p_agent, int t, int service_flag = 0)
{

	if (p_agent->m_PassengerList.size() == 0)
		return; 

	for (auto i = p_agent->m_PassengerList.begin(); i != p_agent->m_PassengerList.end(); i++)
	{
		int p = (*i);  // obtain pax id
		int p_seq_no = g_map_agent_id_to_agent_vector_seq_no[p];
		CAgent* p_pax = &(g_agent_vector[p_seq_no]);

		if(p_pax->m_current_link_seq_no <=p_pax->path_link_seq_no_vector.size() - 2)
		{

			// comments, there is a possibility that a pax without the last link different from the vehicle path, does not exit from the vehicle 

		p_pax->m_Veh_LinkDepartureTime[p_pax->m_current_link_seq_no] = t;  // current link's TD
		p_pax->m_Veh_LinkArrivalTime[p_pax->m_current_link_seq_no + 1] = t; // next link's TA
		p_pax->m_Veh_LinkDepartureTime[p_pax->m_current_link_seq_no + 1] = p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no + 1];  // copy value of TD on next link from vehicle

										
		if(service_flag == -1)  // drop off
		{		
		int next_pax_link_seq_no = p_pax->path_link_seq_no_vector[p_pax->m_current_link_seq_no + 1];
		int next_vehicle_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];

		if (next_pax_link_seq_no != next_vehicle_link_seq_no && p_pax->m_bMoveable == false)
								{  // drop off pax remove it from the list
	
									
									
									//update pax's TA and TD
									p_pax->m_bMoveable = true;

								}

		}

		if (service_flag == 1  && p_pax->m_bMoveable == true)  // pick up
		{
			p_pax->m_bMoveable = false;

		}
		p_pax->m_current_link_seq_no += 1;  // move to next link of pax too

		}

	}

	// remove pax from the list at end of iteration through all pax 
	//
	for (auto i = p_agent->m_PassengerList.begin(); i != p_agent->m_PassengerList.end();)
	{
		int p = (*i);  // obtain pax id
		int p_seq_no = g_map_agent_id_to_agent_vector_seq_no[p];
		CAgent* p_pax = &(g_agent_vector[p_seq_no]);

		if (p_pax->m_bMoveable == true)
		{
			i = p_agent->m_PassengerList.erase(i);


		}
		else
		{
			++i;
		}

	}
}

NetworkForSP* pNetworkForSP = NULL;
STSNetwork* pSTSNetwork = NULL;
list<CAgentElement> g_agent_pointer_list;  // ready to active, and still in the network

void g_TrafficSpaceTimeTrajectorySimulation()
{
	bool bNFDFlag = true;
	// given p_agent->path_link_seq_no_vector path link sequence no for each agent


	for (int l = 0; l < g_number_of_links; l++)
	{
		g_link_vector[l].AllocateMemory();
		g_link_vector[l].ResetMOE();

	}
	// simulation
	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		CAgent* p_agent = &(g_agent_vector[a]);
		p_agent->departure_time_in_min = p_agent->departure_time_in_min;
		p_agent->AllocateMemory();
	}

	int current_active_agent_id = 0;


	int number_of_threads = omp_get_max_threads();


	// use absoluate time scale
	for (int t = g_Simulation_StartTimeInMin*60/ g_number_of_seconds_per_interval; t < g_number_of_simulation_intervals; t+= 1)  // first loop for time t
	{
		int time_in_min = t*g_number_of_seconds_per_interval / 60;  // force to be integer values

//		if (t % (g_number_of_intervals_per_min*5) == 0)
//			cout << "simulation time = " << t / 10 << " min, with " << g_agent_pointer_list.size() << " agents" << endl;

		if (t % g_number_of_intervals_per_min == 0)  /// x interals every min 
		{

			for (int a = current_active_agent_id; a < g_agent_vector.size(); a++)
			{
				CAgent* p_agent = &(g_agent_vector[a]);
				if (t <= p_agent->departure_time_in_simulation_interval && p_agent->departure_time_in_simulation_interval < t + g_number_of_intervals_per_min)  // PER MIN
				{
					p_agent->m_bGenereated = true;

					CAgentElement element;
					element.agent_no = a;
					element.bActive = true;
					g_agent_pointer_list.push_back(element);

					if (p_agent->path_link_seq_no_vector.size() > 0)  // increase the arrival counter of the first link of the vehicle by 1
					{
						p_agent->m_Veh_LinkArrivalTime[0] = p_agent->departure_time_in_simulation_interval;

						int FirstLink = p_agent->path_link_seq_no_vector[0];

						p_agent->m_Veh_LinkDepartureTime[0] = p_agent->m_Veh_LinkArrivalTime[0] + g_link_vector[FirstLink].free_flow_travel_time_in_simu_interval;

						// update A counts for first link
						g_link_vector[FirstLink].m_CumulativeArrivalCount += 1;
     					g_link_vector[FirstLink].m_LinkCumulativePerMinArrival[time_in_min] = g_link_vector[FirstLink].m_CumulativeArrivalCount;

					}

					current_active_agent_id = a+1; //move to the next fresh agent id as active id
				}
				else  // if we encounter an agent with departure time later than the current time t, break from the loop
				{
					break;
				}
			}

		}


#pragma omp parallel for 
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)  // virutal loop for different processors
		{

			for (auto it = g_agent_pointer_list.begin(); it != g_agent_pointer_list.end(); ++it) // second loop for a
			{
				CAgent* p_agent = &(g_agent_vector[(*it).agent_no]);

				if (p_agent->m_bGenereated == true &&
					p_agent->m_bCompleteTrip == false &&
					p_agent->m_bMoveable == true /* vehicle or walking pax */ &&
					p_agent-> agent_id%number_of_threads == ProcessID &&
					p_agent->m_current_link_seq_no < p_agent->m_path_link_seq_no_vector_size
					)
				{
					if (p_agent->agent_id == 4 && p_agent->m_current_link_seq_no >= 0)
					{
						TRACE("");
					}


					if (p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no] == t)  // ready to move to the next link
					{
						// check if the current link has sufficient capacity 
						int link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no];
						CLink* pCurrentLink = &(g_link_vector[link_seq_no]);
						/*condition 1:  moving link*/	if (pCurrentLink->service_type == 0)
						{
							if (pCurrentLink->m_LinkOutFlowCapacity[t] >= 1)  // point queue model 
							{
								if (p_agent->m_current_link_seq_no == p_agent->path_link_seq_no_vector.size() - 1)
								{// end of path

									p_agent->m_bCompleteTrip = true;
									(*it).bActive = false;  // mark inactive element in the queue

									// update D counts for last link
									pCurrentLink->m_CumulativeDepartureCount += 1;
									pCurrentLink->m_LinkCumulativePerMinDeparture[time_in_min] = pCurrentLink->m_CumulativeDepartureCount;

								}
								else
								{ // with capacity, but agent does not complete the trip yet, continue to move

									int next_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];

									p_agent->m_Veh_LinkArrivalTime[p_agent->m_current_link_seq_no + 1] = t;  // t is the simulation time interval
									p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no + 1] = t + g_link_vector[next_link_seq_no].free_flow_travel_time_in_simu_interval;

									g_UpdatePassengerStateInVehicle(p_agent, t, 0);
									

									// update D counts for this link
									pCurrentLink->m_CumulativeDepartureCount += 1;
									pCurrentLink->m_LinkCumulativePerMinDeparture[time_in_min] = pCurrentLink->m_CumulativeDepartureCount;

									// update A counts for next link
#pragma omp critical
									{
										g_link_vector[next_link_seq_no].m_CumulativeArrivalCount += 1;
										g_link_vector[next_link_seq_no].m_LinkCumulativePerMinArrival[time_in_min] = g_link_vector[next_link_seq_no].m_CumulativeArrivalCount;
									}

								}

								//move
								p_agent->m_current_link_seq_no += 1;
								pCurrentLink->m_LinkOutFlowCapacity[t] -= 1;


							}
							else  // no outflow capacity ==0, cause delay
							{
								pCurrentLink->m_CumulativeVirtualDelayCount += 1; // +1 means add one unit of simulation time interval of delay
								pCurrentLink->m_LinkCumulativePerMinVirtualDelay[t] = pCurrentLink->m_CumulativeVirtualDelayCount;

								p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no] = t + 1;  // move to the next time interval to wait
							}
						}

						/*condition 2:  dropoff link*/	if (pCurrentLink->service_type == -1)
						{
							int next_vehicle_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];

							p_agent->m_Veh_LinkArrivalTime[p_agent->m_current_link_seq_no + 1] = t;
							p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no + 1] = t + g_link_vector[next_vehicle_link_seq_no].free_flow_travel_time_in_simu_interval;

							g_UpdatePassengerStateInVehicle(p_agent, t, -1);

							//move
							p_agent->m_current_link_seq_no += 1;
							pCurrentLink->m_LinkOutFlowCapacity[t] -= 1;

						}

							/*condition 3:  pick up link*/	if (pCurrentLink->service_type == 1)
							{
								if (p_agent->agent_type == 1 && p_agent->m_bMoveable == 1)  // pax: can enter link waiting queue
								{
									pCurrentLink->m_waiting_traveler_queue.push_back(p_agent->agent_id);

								}

								if (p_agent->agent_type == 2)  // vehicle: can pick up travelers from the link waiting queue
								{
									

									int remaining_seat_capacity = p_agent->GetRemainingCapacity();
									for (int p_ready = 0; p_ready < remaining_seat_capacity; p_ready++)
									{
										if (pCurrentLink->m_waiting_traveler_queue.size() == 0)
										{
											break;// no traveler in queue, no need to continue
										}

											int p = pCurrentLink->m_waiting_traveler_queue.front();
											pCurrentLink->m_waiting_traveler_queue.pop_front();
											p_agent->Pickup(p);


									}  // end of pick up process

									int next_vehicle_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];

									p_agent->m_Veh_LinkArrivalTime[p_agent->m_current_link_seq_no + 1] = t;
									p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no + 1] = t + g_link_vector[next_vehicle_link_seq_no].free_flow_travel_time_in_simu_interval;

									g_UpdatePassengerStateInVehicle(p_agent,t, 1);


									//move
									p_agent->m_current_link_seq_no += 1;
									pCurrentLink->m_LinkOutFlowCapacity[t] -= 1;


								}


							} // conditions

						
					}  // departure time events
				}  // active agent

			}  // agent pointers

	

		}  // end for processor
		   // clean up list 

		for (auto i = g_agent_pointer_list.begin(); i != g_agent_pointer_list.end();)
		{
			if ((*i).bActive == false)
			{
				i = g_agent_pointer_list.erase(i);
				
				
			}
			else
			{
				++i;
			}
		}

	}
}

void g_TrafficNFDSimulation()
{
	// given p_agent->path_link_seq_no_vector path link sequence no for each agent
	cout << "max_number_of_links_per_path =" << max_number_of_links_per_path << endl;
	cout << "allocating link memory" << endl;

	for (int l = 0; l < g_number_of_links; l++)
	{
		g_link_vector[l].AllocateMemory();
		g_link_vector[l].ResetMOE();

	}
	// simulation
	cout << "start allocating memory for agents.." << endl;

	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		CAgent* p_agent = &(g_agent_vector[a]);
		p_agent->AllocateMemory(0);
	}

	list<CAgentElement> g_agent_pointer_list;  // ready to active, and still in the network
	int current_active_agent_id = 0;


	//int number_of_threads = omp_get_max_threads();  // available CPU processors per min
	int number_of_threads = 8;

	int cumulative_arrival_count = 0;
	int cumulative_departure_count = 0;
	cout << "start NFD simulation.." << endl;

	// use absoluate time scale
	for (int t = g_Simulation_StartTimeInMin * 60 / g_number_of_seconds_per_interval; t < g_number_of_simulation_intervals; t += g_number_of_intervals_per_min)  // first loop for time t
	{
		int time_in_min = t*g_number_of_seconds_per_interval / 60;  // force to be integer values

		if (t % 50 == 0)
			cout << "simulation time = " << t / 10 << " min, with CA: " << current_active_agent_id << " and current: " << g_agent_pointer_list.size() << " agents" << endl;

#pragma omp parallel for 
		for (int l = 0; l < g_number_of_links; l++)
		{
			g_link_vector[l].ResetPerMinADCounts(number_of_threads);
		}
		
		// this must be a sequential process to scan the departure time of each agent in the agent vector
			for (int a = current_active_agent_id; a < g_agent_vector.size(); a++) // go through all vehicles
				{
					CAgent* p_agent = &(g_agent_vector[a]);
					if (t <= p_agent->departure_time_in_simulation_interval && p_agent->departure_time_in_simulation_interval < t + g_number_of_intervals_per_min)  // PER NFD simulation time period
					{
						p_agent->m_bGenereated = true;

						CAgentElement element;
						element.agent_no = a;
						element.bActive = true;
						g_agent_pointer_list.push_back(element);

						if (p_agent->path_link_seq_no_vector.size() > 0)  // increase the arrival counter of the first link of the vehicle by 1
						{
							p_agent->m_Veh_LinkArrivalTime[0] = p_agent->departure_time_in_simulation_interval;

							int FirstLink = p_agent->path_link_seq_no_vector[0];

							p_agent->m_Veh_LinkDepartureTime[0] = p_agent->m_Veh_LinkArrivalTime[0] + g_link_vector[FirstLink].free_flow_travel_time_in_simu_interval;

							// update A counts for first link
							g_link_vector[FirstLink].m_CumulativeArrivalCount += 1;
							int time_in_min = p_agent->departure_time_in_min;  //  p_agent->departure_time_in_min might vary within the NSD simulation time period// g_agent_vector is sequential in terms of departure time
							g_link_vector[FirstLink].m_LinkCumulativePerMinArrival[time_in_min] = g_link_vector[FirstLink].m_CumulativeArrivalCount;

						}

						current_active_agent_id = a + 1; //move to the next fresh agent id as active id
					}
					else  // if we encounter an agent with departure time later than the current time t, break from the loop
					{
						break;
					}
				}
		

#pragma omp parallel for 
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)  // virutal loop for different processors  so that, we have a processor for each worker
		{
			for (auto it = g_agent_pointer_list.begin(); it != g_agent_pointer_list.end(); ++it) // second loop for a
			{
				CAgent* p_agent = &(g_agent_vector[(*it).agent_no]);

				if (p_agent->m_bGenereated == true &&
					p_agent->m_bCompleteTrip == false &&
					p_agent->agent_id %number_of_threads == ProcessID && //* different sets of agents are assigned to different processors*/
					p_agent->m_bMoveable == true /* vehicle or walking pax */
					)
				{


					while (p_agent->m_current_link_seq_no < p_agent->m_path_link_seq_no_vector_size)
					{
						// check if the current link has sufficient capacity 
						
							if (p_agent->m_current_link_seq_no + 1 < p_agent->path_link_seq_no_vector.size() -1  && p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no] <= t + g_number_of_intervals_per_min)
							{
								int next_link_seq_no = p_agent->path_link_seq_no_vector[p_agent->m_current_link_seq_no + 1];
								p_agent->m_Veh_LinkArrivalTime[p_agent->m_current_link_seq_no + 1] = p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no];
								p_agent->m_Veh_LinkDepartureTime[p_agent->m_current_link_seq_no + 1] = p_agent->m_Veh_LinkArrivalTime[p_agent->m_current_link_seq_no + 1] + g_link_vector[next_link_seq_no].free_flow_travel_time_in_simu_interval;

								// update D counts for this link
								g_link_vector[p_agent->m_current_link_seq_no].m_CumulativeDepartureCountPerMinPerProcessor[ProcessID] += 1;  // update count at this processor of this link
																																			 // update D counts for this link
								// update A counts for next link
								g_link_vector[p_agent->m_current_link_seq_no+1].m_CumulativeArrivalCountPerMinPerProcessor[ProcessID] += 1;  // update count at this processor of this link

							}
							else
							{
								if (p_agent->m_current_link_seq_no + 1 >= p_agent->path_link_seq_no_vector.size() - 1)
								{
									p_agent->m_bCompleteTrip = true;
									(*it).bActive = false;  // mark inactive element in the queue
									// update D counts for last link
									g_link_vector[p_agent->m_current_link_seq_no].m_CumulativeDepartureCountPerMinPerProcessor[ProcessID] += 1;  // update count at this processor of this link

								}
								break;
							}
						//move
						p_agent->m_current_link_seq_no += 1;
					}

				}  // active agent

			}  // agent pointers
			}  // processor

			for (auto i = g_agent_pointer_list.begin(); i != g_agent_pointer_list.end();)
			{
				if ((*i).bActive == false)
				{
					i = g_agent_pointer_list.erase(i);
					
					
				}
				else
				{
					++i;
				}
			}

			// update AD count for all links
#pragma omp parallel for 
			for (int l = 0; l < g_number_of_links; l++)
			{
				g_link_vector[l].TallyPerMinADCounts(number_of_threads, time_in_min);
			}
		
	}  // for time t
}


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
			g_node_vector[i].m_SeqNo4OriginNode = number_of_origin_nodes;
			number_of_origin_nodes++;
		}

	}


	int number_of_threads = max(1,number_of_origin_nodes);

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

		int thread_no = g_node_vector[internal_origin_node_seq_no].m_SeqNo4OriginNode;
		pNetworkForSP[thread_no].m_agent_vector.push_back(a);
		}
	}
	return max(1,number_of_threads);
}

int g_AssignThreadsBasedonOriginNodeAndDepartureTimeInterval()
{

	// step 1: scan all agents
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
			g_node_vector[i].m_SeqNo4OriginNode = number_of_origin_nodes;
			number_of_origin_nodes++;
		}

	}

	int number_of_departure_time_intervals = 100;
	int number_of_threads = max(1, number_of_origin_nodes)*number_of_departure_time_intervals;

	// step 2: create STS networks
	pSTSNetwork = new STSNetwork[number_of_threads]; // create n copies of network, each for a subset of agents to use 

	cout << " number of threads = " << number_of_threads << endl;

	cout << " end of network memory allocation. " << endl;

	number_of_origin_nodes = 0;
	for (int i = 0; i < g_node_vector.size(); i++)
	{
		if (g_node_vector[i].bOriginNode_ForAgents == true)
		{
			number_of_origin_nodes++;
		}

	}

	// step 3: assign all agents to the corresponding thread
	for (int a = 0; a < g_agent_vector.size(); a++)  //assign all agents to the corresponding thread
	{
		CAgent* p_agent = &(g_agent_vector[a]);
		if (p_agent->fixed_path_flag != 1)
		{
			int internal_origin_node_seq_no = g_internal_node_seq_no_map[p_agent->origin_node_id];  // map external node number to internal node seq no. 

			int departure_time_index = p_agent->departure_time_in_min / 15;
			int thread_no = (g_node_vector[internal_origin_node_seq_no].m_SeqNo4OriginNode*number_of_departure_time_intervals) + departure_time_index;
			// thread no = origin node seq. no * DEPARTURE TIME_SIZE + depature time index
			// 2 dimension to 1 dimension

			pSTSNetwork[thread_no]. m_origin_node = internal_origin_node_seq_no;
			pSTSNetwork[thread_no].m_departure_time_beginning = departure_time_index * 15;
			pSTSNetwork[thread_no].m_arrival_time_ending = pSTSNetwork[thread_no].m_departure_time_beginning + 200;

			pSTSNetwork[thread_no].m_agent_vector.push_back(a);
		}
	}

	for (int i = 0; i < number_of_threads; i++)
	{
		pSTSNetwork[i].m_threadNo = i;   // each thread/network has its own thread number. // each network has its own label cost vector for shortest path calculation
		if(pSTSNetwork[i].m_agent_vector.size()>0)
		{ 
		pSTSNetwork[i].AllocateSTSMemory(g_number_of_nodes, 1440, _MAX_STATES);
		}
	}
	return max(1, number_of_threads);
}


void g_OutputVehicleTrajectory()
{
	for (int i = 0; i < g_agent_vector.size(); i++)
	{
		for (int j = 0; j < g_agent_vector[i].tsview_timestamp_vector.size(); j++)
		{
			int local_node = g_agent_vector[i].path_node_id_vector[j] % 1000;
			fprintf(g_pTSViewOutput, "%d,%4.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", g_agent_vector[i].agent_id, g_agent_vector[i].tsview_timestamp_vector[j]*10,
				2000, 2000, 1, local_node, 1, 1000, 10, 5, 2, 1, 1, 1, 0, 0, 0, 0);
		}
	}
}

void g_OutputLinkMOE()
{
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

	if(g_generate_link_TDMOE_csv==1)
	{ 

	FILE* st = NULL;
	st = fopen("output_LinkTDMOE.csv", "w");
	if (st == NULL)
	{
		cout << "File output_LinkTDMOE.csv cannot be opened." << endl;
		g_ProgramStop();
	}


	fprintf(st, "from_node_id,to_node_id,timestamp_in_min,travel_time_in_min,link_volume_in_veh_per_hour_for_all_lanes,density_in_veh_per_distance_per_lane,speed,cumulative_arrival_count,cumulative_departure_count\n");

	if(g_TrafficFlowModel>0)
	{ 

	for (int l = 0; l < g_link_vector.size(); l++) //Initialization for all nodes
	{
		CLink* pLink = &(g_link_vector[l]);

		for (int t = g_Simulation_StartTimeInMin; t < g_Simulation_EndTimeInMin; t++)
		{
			int time_in_simulation_interval = t * 60 / g_number_of_seconds_per_interval;

			float avg_travel_time_in_min = pLink->free_flow_travel_time_in_min + pLink->get_avg_delay_in_min(t, 1);
			fprintf(st, "%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
				g_node_vector[pLink->from_node_seq_no].node_id,
				g_node_vector[pLink->to_node_seq_no].node_id,
				t,
				avg_travel_time_in_min,
				pLink->get_link_in_flow_per_min(t)*60.0,
				pLink->get_number_of_vehicles(t) / max(0.0001, pLink->length * pLink->number_of_lanes),
				pLink->length / max(0.001, avg_travel_time_in_min) / 60.0,
				pLink->m_LinkCumulativePerMinArrival[t],
				pLink->m_LinkCumulativePerMinDeparture[t]);
		}

	}
	}
	fclose(st);
	}
}

void g_TrafficAssignmentSimulationProcess()
{

	int number_of_threads = g_AssignThreadsBasedonOriginNodes();

	for (int assignment_iteration_no = 0; assignment_iteration_no < g_NumberOfIterations; assignment_iteration_no++)
	{
		cout << "Assignment Interval = %d" << assignment_iteration_no << endl;

		for (int l = 0; l < g_number_of_links; l++)
		{
			g_link_vector[l].CalculateBRPFunction();  //based on the g_link_vector[l].flow_volume
		}

#pragma omp parallel for  // step 3: C++ open mp automatically create n threads., each thread has its own computing thread on a cpu core 
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{
			cout << "Processor " << ProcessID << " of "<< number_of_threads << " threads is calculating shortest paths" << endl;

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

				//fprintf(g_pFileDebugLog, "\iteration = %d,processor = %d, link no= %d, volume = %f, total volume = %f\n",
				//	assignment_iteration_no, ProcessID, l, pNetworkForSP[ProcessID].m_link_volume_array[l], g_link_vector[l].flow_volume);

			}
		}



	}
	
	if(g_TrafficFlowModel==1)
		g_TrafficSpaceTimeTrajectorySimulation();

	if (g_TrafficFlowModel == 2)
		g_TrafficNFDSimulation();

	cout << "outputing files... " << endl;

	g_OutputLinkMOE();

	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		CAgent* p_agent = &(g_agent_vector[a]);

		if (p_agent->fixed_path_flag != 1)
		{
			std::reverse(std::begin(p_agent->path_node_id_vector),
				std::end(p_agent->path_node_id_vector));
		}
	}



	if(g_generate_agent_csv==1)
	{ 
	FILE* g_pFileAgent = NULL;
	g_pFileAgent = fopen("output_agent.csv", "w");
	if (g_pFileAgent == NULL)
	{
		cout << "File output_agent.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	else
	{

		fprintf(g_pFileAgent, "agent_id,agent_type,origin_node_id,destination_node_id,vehicle_seat_capacity,PCE,departure_time_in_min,travel_time_in_min,arrival_time_in_min,fixed_path_flag,path_index,number_of_nodes,path_node_sequence,path_time_sequence_in_min,");

		if (g_number_of_seconds_per_interval == 1)
			fprintf(g_pFileAgent, "path_time_sequence_in_sec,");

		fprintf(g_pFileAgent, "\n");



		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);
			float arrival_time_in_min = p_agent->m_Veh_LinkDepartureTime[p_agent->path_link_seq_no_vector.size() - 1] * g_number_of_seconds_per_interval / 60.0;
			p_agent->travel_time_in_min = arrival_time_in_min- p_agent->departure_time_in_min;

			fprintf(g_pFileAgent, "%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,",
				p_agent->agent_id,
				p_agent->agent_type,
				p_agent->origin_node_id,
				p_agent->destination_node_id,
				p_agent->vehicle_seat_capacity,
				p_agent->PCE_factor,
				p_agent->departure_time_in_min,
				p_agent->travel_time_in_min, 
				arrival_time_in_min,
				p_agent->fixed_path_flag,
				p_agent->path_index,
			    p_agent->path_node_id_vector.size()


			);

			for (int i = 0; i < p_agent->path_node_id_vector.size(); i++)
			{
				fprintf(g_pFileAgent, "%d;", p_agent->path_node_id_vector[i]);
			}

			fprintf(g_pFileAgent, ",");

			if (g_TrafficFlowModel > 0)
			{

				for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
				{
					float TA = p_agent->m_Veh_LinkArrivalTime[i] * g_number_of_seconds_per_interval / 60.0;
					fprintf(g_pFileAgent, "%.1f:", TA);
					p_agent->tsview_timestamp_vector.push_back(TA);
				}

				if (p_agent->path_link_seq_no_vector.size() >= 1)
				{

					int size = p_agent->path_link_seq_no_vector.size();

					float TD = p_agent->m_Veh_LinkDepartureTime[p_agent->path_link_seq_no_vector.size()-1] * g_number_of_seconds_per_interval / 60.0;
					fprintf(g_pFileAgent, "%.1f,", TD);
					p_agent->tsview_timestamp_vector.push_back(TD);
				}

			}


		if(g_number_of_seconds_per_interval == 1)
		{ 
			fprintf(g_pFileAgent, ",");

			for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
			{
				int TA = p_agent->m_Veh_LinkArrivalTime[i] * g_number_of_seconds_per_interval;
				fprintf(g_pFileAgent, "%d:", TA);
			}

			if(p_agent->path_link_seq_no_vector.size()>=1)
			{ 

				int size = p_agent->path_link_seq_no_vector.size();

			int TD = p_agent->m_Veh_LinkDepartureTime[size-1] * g_number_of_seconds_per_interval;
			fprintf(g_pFileAgent, "%d,", TD);
			}


		}

			fprintf(g_pFileAgent, "\n");

		}
		//
		fclose(g_pFileAgent);
	}

	}

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

	// step 2: call 	g_TrafficSpaceTimeTrajectorySimulation(); 
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

		// step 2: call 	g_TrafficSpaceTimeTrajectorySimulation(); 
		// based on given input_sensor speed, simulate vehicles (with flag_to_be_generated == 1) along their physical paths, propagate space time trajectories : output: simulated link flow and density (travel time with waiting time subject to capacity constraints)
		// capacity constraints are enforced after time current timestamp, for different future scenarios, e.g. different work zones, ramp metering or signal control methods 

		g_TrafficSpaceTimeTrajectorySimulation();

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

bool g_Optimization_Lagrangian_Relaxation_Method()
{

	g_AssignThreadsBasedonOriginNodeAndDepartureTimeInterval();

	for (int l = 0; l < g_link_vector.size(); l++)
	{
		CLink* pLink = &(g_link_vector[l]);
		pLink->Setup_State_Dependent_Data_Matrix();
	}

	float g_best_upper_bound = 99999;
	float g_best_lower_bound = -99999;

	int g_number_of_LR_iterations = 10;
	float g_minimum_subgradient_step_size = 0.01;
	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{

		cout << "Lagrangian Iteration " << LR_iteration << "/" << g_number_of_LR_iterations << endl;

		//step 1: check time_dependent flow rates
		//	setup  time_dependent_flow_volume_vector[_MAX_NUMBER_OF_TIME_INTERVALS];
		//	setup state_dependent_time_dependent_LR_multiplier_matrix[_MAX_NUMBER_OF_TIME_INTERVALS][_MAX_STATES];

		// for each link

		for (int l = 0; l < g_link_vector.size(); l++)
		{
			CLink* pLink = &(g_link_vector[l]);

			for (int t = g_Simulation_StartTimeInMin; t < g_Simulation_EndTimeInMin; t++)
			{

				float OverCapacityVolume = max(0, pLink->time_dependent_flow_volume_vector[t] - pLink->link_capacity_per_min);

				float StepSize = 1;
				float OverCapacityPenality = 100;

				if (LR_iteration == 0)
				{
					StepSize = OverCapacityPenality; // initial value from base penalty
				}
				else
				{
					StepSize = OverCapacityPenality / (LR_iteration + 1.0f);
				}

				if (StepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
				{
					StepSize = g_minimum_subgradient_step_size;
				}

				pLink->state_dependent_time_dependent_LR_multiplier_matrix[t][0] =
					pLink->state_dependent_time_dependent_LR_multiplier_matrix[t][0] + StepSize*OverCapacityVolume;

			}  // for time t

		} // for link

		  // Lower bound 
		int number_of_threads = g_AssignThreadsBasedonOriginNodes();

#pragma omp parallel for  // step 3: C++ open mp automatically create n threads., each thread has its own computing thread on a cpu core 
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{
			cout << "Processor " << ProcessID << " of " << number_of_threads << " threads is calculating shortest paths" << endl;

			pSTSNetwork[ProcessID].find_STS_path_for_agents_assigned_for_this_thread(number_of_threads, LR_iteration);
		}
	}

	//for each lagrangian relaxation iteration

	cout << "End of Lagrangian Iteration Process " << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;

	
	return true;
}




int main(int argc, TCHAR* argv[], TCHAR* envp[])
{

	// definte timestamps
	clock_t start_t, end_t, total_t;
	int i;

	start_t = clock();
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

	 g_pTSViewOutput = fopen("output_trajectory.csv", "w");
	 if (g_pTSViewOutput == NULL)
	 {
		 cout << "File output_trajectory.csv cannot be opened." << endl;
		 g_ProgramStop();
	 }

	 // step 1: read input data of network and demand agent
	g_ReadInputData();   

	for (int o = 0; o < g_number_of_zones; o++)
		for (int d = 0; d < g_number_of_zones; d++)
		{
			g_OD_loading_multiplier[o][d] = g_loading_multiplier;
		}


	// step 2: call different major modeling methods

	// mode 1: void g_TrafficAssignmentSimulationProcess()

	 g_TrafficAssignmentSimulationProcess();
	// mode 2: g_AgentBasedODMEProcess()

	// method 3: LR based optimization
//	g_Optimization_Lagrangian_Relaxation_Method();
	
	end_t = clock();

	total_t = (end_t - start_t);

	cout << "CPU Running Time = " << total_t << " milliseconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %.3f seconds\n", total_t/1000.0);
	fprintf(g_pFileOutputLog, "CPU Running Time =,%.3f, seconds\n", total_t / 1000.0);

	fclose(g_pFileOutputLog);
	fclose(g_pFileDebugLog);

	g_OutputVehicleTrajectory();
	fclose(g_pTSViewOutput);

	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;
	cout << "done." << endl;
	delete[] pNetworkForSP;

	g_node_vector.clear();
	g_link_vector.clear();

	getchar();

	return 1;
}



