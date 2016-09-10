#pragma once


#include "stdafx.h"
#include <vector>
using namespace std; 
#define _MAX_LABEL_COST 99999

#define _MAX_NUMBER_OF_NODES 50000
#define _MAX_NUMBER_OF_LINKS 90000
#define _MAX_NUMBER_OF_TIME_INTERVALS 1400

#define _MAX_NUMBER_OF_VEHICLES 200
#define _MAX_NUMBER_OF_PASSENGERS 70
#define _MAX_NUMBER_OF_STATES 3000
extern int g_number_of_physical_vehicles;
extern int g_number_of_passengers;


#define _MAX_NUMBER_OF_OUTBOUND_NODES 10
extern void g_STVComputing();
extern void g_DynamicProgramming_transition_arc(int k);
extern int g_number_of_time_intervals;

extern int g_vehicle_path_node_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_vehicle_path_link_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_vehicle_path_time_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES];
extern float g_external_link_time_dependent_toll[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

extern float g_optimal_time_dependenet_label_correcting(float arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS],
	int origin_node, int departure_time_beginning, int departure_time_ending, int destination_node, int arrival_time_beginning, int arrival_time_ending,
	int &path_number_of_nodes,
	int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], int path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int travel_time_calculation_flag,
	float &travel_time_return_value);

extern void g_TrafficSimulation(int SimulationStartTime, int SimulationEndTime, int b_spatial_capacity_flag);
extern int g_Brand_and_Bound();


extern int g_outbound_node_size[_MAX_NUMBER_OF_NODES];
extern int g_node_passenger_id[_MAX_NUMBER_OF_NODES];
extern int g_node_passenger_origin_flag[_MAX_NUMBER_OF_NODES];


extern int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
extern int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
extern int g_activity_node_flag[_MAX_NUMBER_OF_NODES];
extern int g_activity_node_ending_time[_MAX_NUMBER_OF_NODES];
extern int g_activity_node_starting_time[_MAX_NUMBER_OF_NODES];

extern int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
extern int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
extern int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

extern float** g_passenger_activity_node_multiplier;

extern int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
extern float g_link_free_flow_travel_time_float_value[_MAX_NUMBER_OF_LINKS];

extern float g_link_link_length[_MAX_NUMBER_OF_LINKS];
extern int g_link_number_of_lanes[_MAX_NUMBER_OF_LINKS];
extern int g_link_mode_code[_MAX_NUMBER_OF_LINKS];
extern float g_link_capacity_per_time_interval[_MAX_NUMBER_OF_LINKS];
extern float g_link_jam_density[_MAX_NUMBER_OF_LINKS];
extern int g_link_service_code[_MAX_NUMBER_OF_LINKS];

extern float g_link_speed[_MAX_NUMBER_OF_LINKS];
extern int g_link_from_node_number[_MAX_NUMBER_OF_LINKS];
extern int g_link_to_node_number[_MAX_NUMBER_OF_LINKS];

extern float g_VOIVTT_per_hour[_MAX_NUMBER_OF_VEHICLES];
extern float g_VOWT_per_hour[_MAX_NUMBER_OF_VEHICLES];

extern int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES];

extern int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
extern int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];

extern int g_passenger_origin_node[_MAX_NUMBER_OF_PASSENGERS];  // traveling passengers
extern int g_passenger_departure_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
extern int g_passenger_departure_time_ending[_MAX_NUMBER_OF_PASSENGERS];
extern int g_passenger_destination_node[_MAX_NUMBER_OF_PASSENGERS];
extern int g_passenger_dummy_destination_node[_MAX_NUMBER_OF_PASSENGERS];

extern int g_passenger_arrival_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
extern int g_passenger_arrival_time_ending[_MAX_NUMBER_OF_PASSENGERS];
extern float g_passenger_origin_multiplier[_MAX_NUMBER_OF_PASSENGERS];
extern float g_passenger_destination_multiplier[_MAX_NUMBER_OF_PASSENGERS] ;

//float g_passenger_request_cancelation_cost[_MAX_NUMBER_OF_PASSENGERS] = { 0 };



extern int g_vehicle_capacity[_MAX_NUMBER_OF_VEHICLES];

extern int g_passenger_assigned_vehicle_id[_MAX_NUMBER_OF_PASSENGERS];
extern int g_passenger_path_node_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_passenger_path_link_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_passenger_path_time_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_passenger_path_number_of_nodes[_MAX_NUMBER_OF_PASSENGERS];

extern int g_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_path_number_of_nodes;
extern float g_path_travel_time[_MAX_NUMBER_OF_VEHICLES];

extern int g_number_of_links;
extern int g_number_of_nodes;
extern int g_number_of_physical_nodes;

extern int g_number_of_time_intervals;

extern int g_number_of_vehicles;
extern int g_number_of_physical_vehicles;

extern int g_number_of_toll_records;

extern int g_number_of_LR_iterations;
extern int g_minimum_subgradient_step_size;

extern int g_shortest_path_debugging_flag;
extern float g_waiting_time_ratio;
extern float g_dummy_vehicle_cost_per_hour;

extern bool g_no_changed_route_for_passengers_flag;
extern bool g_no_capacity_multiplier_flag;

extern float g_travel_time_budget;
extern float g_idle_vehicle_benefit;


extern float*** l_state_node_label_cost;
extern int*** l_state_node_predecessor;
extern int*** l_state_time_predecessor;
extern int*** l_state_carrying_predecessor;

//parallel computing 
extern float**** lp_state_node_label_cost;
extern int**** lp_state_node_predecessor;
extern int**** lp_state_time_predecessor;
extern int**** lp_state_carrying_predecessor;

extern float**** lp_backward_state_node_label_cost;

extern float** g_node_to_node_shorest_travel_time;

extern float*** g_v_arc_cost;
extern float*** g_v_to_node_cost_used_for_upper_bound;
extern float*** g_v_to_node_cost_used_for_lower_bound;
extern float*** g_v_vertex_waiting_cost;

extern class CVRState;  //class for vehicle scheduling states;

class V2PAssignment
{
public:
	int input_assigned_vehicle_id;
	std::vector <int> input_prohibited_vehicle_id_vector;
	std::vector <int> output_competting_vehicle_id_vector;


	V2PAssignment()
	{
		input_assigned_vehicle_id = -1;
	}

	bool AddCompettingVehID(int vehicle_id)
	{
		for (int i = 0; i < output_competting_vehicle_id_vector.size(); i++)  // test if the vehicle id is already in the lest
		{
			if (output_competting_vehicle_id_vector[i] == vehicle_id)
				return false;
		}

		output_competting_vehicle_id_vector.push_back(vehicle_id);

	}
};


class VRP_exchange_data
{
public:
	int BBNodeNo;
	float UBCost;
	float LBCost;

	std::vector <V2PAssignment>  V2PAssignmentVector;

		bool CopyAssignmentInput(std::vector <V2PAssignment> ExternalV2PAssignmentVector)
	{
		for (int p = 0; p <= g_number_of_passengers; p++)
		{
		
			V2PAssignmentVector[p].input_assigned_vehicle_id = ExternalV2PAssignmentVector[p].input_assigned_vehicle_id;
				for (int i = 0; i < ExternalV2PAssignmentVector[p].input_prohibited_vehicle_id_vector.size(); i++)
			{
					V2PAssignmentVector[p].input_prohibited_vehicle_id_vector.push_back(ExternalV2PAssignmentVector[p].input_prohibited_vehicle_id_vector[i]);

			}
		}
		return true;
	}

	bool AddP2VAssignment(int p, int v)
	{

		V2PAssignmentVector[p].input_assigned_vehicle_id = v;
		return true;
	}

	bool AddProhibitedAssignment(int p, std::vector <int> prohibited_vehicle_id_vector)
	{

		V2PAssignmentVector[p].input_prohibited_vehicle_id_vector = prohibited_vehicle_id_vector;
		return true;
	}
	bool reset_output()
	{
		LBCost = -99999;
		UBCost = 99999;
		for (int j = 0; j <= g_number_of_passengers; j++)
		{
			V2PAssignmentVector[j].output_competting_vehicle_id_vector.clear();
		}
		return true;
	}
	bool bV2P_Prohibited(int pax_id, int vehicle_id)
	{

		for (int i = 0; i < V2PAssignmentVector[pax_id].input_prohibited_vehicle_id_vector.size(); i++)
		{
			if (V2PAssignmentVector[pax_id].input_prohibited_vehicle_id_vector[i] == vehicle_id)
				return true;

		}

		return false;

	}
	VRP_exchange_data()
	{
		BBNodeNo = -1;
		LBCost = -99999;
		UBCost = 99999;
		for (int j = 0; j <= g_number_of_passengers; j++)
		{

			V2PAssignment element;
			V2PAssignmentVector.push_back(element);
		}
	}

};
template <typename T>
T **AllocateDynamicArray(int nRows, int nCols, int initial_value = 0)
{
	T **dynamicArray;

	dynamicArray = new (std::nothrow) T*[nRows];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();

	}

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new (std::nothrow) T[nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int j = 0; j < nCols; j++)
		{
			dynamicArray[i][j] = initial_value;
		}
	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray, int nRows, int nCols)
{
	if (!dArray)
		return;

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
		if (x % 1000 ==0)
		{
			cout << "allocating 3D memory for " << x << endl;
		}


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

	for (int x = 0; x < nX; x++)
	for (int y = 0; y < nY; y++)
	for (int z = 0; z < nZ; z++)
	{
		dynamicArray[x][y][z] = 0;
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



template <typename T>
T ****Allocate4DDynamicArray(int nM, int nX, int nY, int nZ)
{
	T ****dynamicArray;

	dynamicArray = new (std::nothrow) T***[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}
	for (int m = 0; m < nM; m++)
	{
		if (m%100 ==0)
		cout << "allocating 4D memory for " << m << endl;

		dynamicArray[m] = new (std::nothrow) T**[nX];

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T*[nY];

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T[nZ];
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					g_ProgramStop();
				}
			}
		}
	}
	return dynamicArray;

}

template <typename T>
void Deallocate4DDynamicArray(T**** dArray, int nM, int nX, int nY)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				delete[] dArray[m][x][y];
			}

			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}

extern void g_ProgramStop();


template <typename T>
T *****Allocate5DDynamicArray(int nM, int nX, int nY, int nZ, int nW)
{
	T *****dynamicArray;

	dynamicArray = new (std::nothrow) T****[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}
	for (int m = 0; m < nM; m++)
	{
		cout << "allocating 5D memory for " << m << endl;

		dynamicArray[m] = new (std::nothrow) T***[nX];

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T**[nY];

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T*[nZ];
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					g_ProgramStop();
				}

				for (int w = 0; w < nW; w++)
				{
					dynamicArray[m][x][y][w] = new (std::nothrow) T[nW];
					if (dynamicArray[m][x][y][w] == NULL)
					{
						cout << "Error: insufficient memory.";
						g_ProgramStop();
					}
				}
			}
		}
	}
	return dynamicArray;

}

template <typename T>
void Deallocate5DDynamicArray(T**** dArray, int nM, int nX, int nY, int nW)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				for (int w = 0; w < nW; w++)
				{
					delete[] dArray[m][x][y][w];
				}

				delete[] dArray[m][x][y];


			}

			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}



extern bool g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables(VRP_exchange_data* local_vrp_data);  // with varaible y only;
