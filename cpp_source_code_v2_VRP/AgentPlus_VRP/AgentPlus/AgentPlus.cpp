// AgentPlus.cpp : Defines the entry point for the console application.
//  Portions Copyright 2010 Xuesong Zhou

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL
//   and further prevent a violation of the GPL.

//Dr.Jianrui Miao <jrmiao@bjtu.edu.cn>
//Pengheng Li <peihengl@asu.edu>
//Monirehalsadat Mahmoudi <mmahmou2@asu.edu>

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of AgentPlus-VRP.

//    AgentPlus-VRP is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    AgentPlus-VRP is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with AgentPlus-VRP.  If not, see <http://www.gnu.org/licenses/>.

//add your names here

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "AgentPlus.h"
#include "CSVParser.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// The one and only application object

CWinApp theApp;
using namespace std;

FILE* g_pFileDebugLog = NULL;

FILE* g_pFileOutputLog = NULL;
FILE* g_pFileAgentPathLog = NULL;




int g_number_of_threads = 1;

int g_node_passenger_id[_MAX_NUMBER_OF_NODES];
int g_node_type[_MAX_NUMBER_OF_NODES];  //key nodeid; 1: pick up, 2: drop off
int g_node_timestamp[_MAX_NUMBER_OF_NODES];  //key nodeid;, values type 1: pick up:ready time, 2: order time
float g_node_baseprofit[_MAX_NUMBER_OF_NODES];  // type 2; = 0
int g_outbound_node_size[_MAX_NUMBER_OF_NODES];  //key nodeid;, values type 1: pick up:ready time, 2: order time


VRP_exchange_data g_VRP_data;

float g_passenger_base_profit[_MAX_NUMBER_OF_PASSENGERS] = { -7 };
float local_vehicle_passenger_additional_profit[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };

float g_passenger_order_time[_MAX_NUMBER_OF_PASSENGERS] = { 0};
int g_passenger_place_time[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_passenger_min_delivery_time[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_passenger_number_of_visits[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_passenger_vehicle_visit_flag[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_VEHICLES] = { 0 };
int g_vehicle_passenger_visit_allowed_flag[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };

int g_max_vehicle_capacity = 1;
int g_number_of_passengers = 0;

int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
float g_link_free_flow_travel_time_float_value[_MAX_NUMBER_OF_LINKS];

int g_link_service_code[_MAX_NUMBER_OF_LINKS] = { 0 };

int g_link_from_node_number[_MAX_NUMBER_OF_LINKS];
int g_link_to_node_number[_MAX_NUMBER_OF_LINKS];

float g_VOIVTT_per_hour[_MAX_NUMBER_OF_VEHICLES];
float g_VOWT_per_hour[_MAX_NUMBER_OF_VEHICLES];

int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };

int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_departure_time[_MAX_NUMBER_OF_VEHICLES] = { 120 };
int g_vehicle_arrival_time[_MAX_NUMBER_OF_VEHICLES] = { 120 };

int g_passenger_origin_node[_MAX_NUMBER_OF_PASSENGERS];  // traveling passengers

int g_vehicle_capacity[_MAX_NUMBER_OF_VEHICLES] = { 1 };

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_physical_nodes = 0;
int g_number_of_time_intervals = 10;

int g_number_of_vehicles = 0;

int g_number_of_LR_iterations = 1;
int g_minimum_subgradient_step_size = 0.01;

int g_shortest_path_debugging_flag = 1;
float g_waiting_time_ratio = 0.005;
float g_dummy_vehicle_cost_per_hour = 100;

float g_travel_time_budget = 100;
float g_idle_vehicle_benefit = -10;

CTime g_SolutionStartTime;


int g_get_link_no_based_on_from_node_to_node(int from_node, int to_node)
{
	if (from_node >= _MAX_NUMBER_OF_NODES)
		return -1;

	if (from_node == to_node)
		return -1;

	//scan outbound links from a upstream node
	for (int i = 0; i < g_outbound_node_size[from_node]; i++)
	{
		if (g_outbound_node_id[from_node][i] == to_node)
			return g_outbound_link_no[from_node][i];
	}
	return -1;
}


void g_ProgramStop()
{

	cout << "Agent+ Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};


class CVSState  //class for vehicle scheduling states
{
public:
	int current_node_id;  // space dimension
	std::map<int, int> passenger_service_state;
	std::map<int, int> passenger_service_time;
	std::map<int, int> passenger_pickup_time;

	std::map<int, int> passenger_carrying_state;
	std::vector<int> m_visit_sequence;  // store nodes
	int m_vehicle_capacity;
	float LabelCost;  // with LR price
	float PrimalLabelCost;  // without LR price
	float TotalDeliveryTime;
	int TotalDeliveried;

	int m_final_arrival_time;   // for ending states
	bool m_bRoute_close;

	CVSState()
	{
		m_final_arrival_time = 0;
		LabelCost = _MAX_LABEL_COST;
		m_vehicle_capacity = 9999;
		m_bRoute_close = false;
	}
	bool IsEmpty()
	{
		if (m_visit_sequence.size() == 0)
			return true;
		else
			return false;
	}
	void Clear()
	{
		m_final_arrival_time = 0;
		LabelCost = _MAX_LABEL_COST;
		m_vehicle_capacity = 9999;

		passenger_service_state.clear();

		passenger_carrying_state.clear();
		passenger_pickup_time.clear();
		passenger_service_time.clear();

		m_visit_sequence.clear();
		m_bRoute_close = false;
	}

	void Copy(CVSState* pSource)
	{
		current_node_id = pSource->current_node_id;
		passenger_service_state.clear();
		passenger_service_state = pSource->passenger_service_state;

		passenger_service_time.clear();
		passenger_service_time = pSource->passenger_service_time;

		passenger_pickup_time.clear();
		passenger_pickup_time = pSource->passenger_pickup_time;

		passenger_carrying_state.clear();
		passenger_carrying_state = pSource->passenger_carrying_state;

		m_visit_sequence.clear();
		m_visit_sequence = pSource->m_visit_sequence;
		m_vehicle_capacity = pSource->m_vehicle_capacity;
		LabelCost = pSource->LabelCost;
		this->TotalDeliveried = pSource->TotalDeliveried;
		this->TotalDeliveryTime = pSource->TotalDeliveryTime;
		this->m_bRoute_close = pSource->m_bRoute_close;
	}
	int GetPassengerServiceState(int passenger_id)
	{
		if (passenger_service_state.find(passenger_id)!= passenger_service_state.end())
			return passenger_service_state[passenger_id];  // 1 or 2
		else
			return 0;
	}


	void StartCarryingService(int passenger_id,int time)
	{
		passenger_carrying_state[passenger_id] = 1;
		passenger_pickup_time[passenger_id] = time;
	}

	void CompleteCarryingService(int passenger_id, int service_time)
	{
		map<int, int>::iterator iter = passenger_carrying_state.find(passenger_id);
		if (iter != passenger_carrying_state.end())
		{
			passenger_carrying_state.erase(iter);
		}

		passenger_service_time[passenger_id] = service_time;
	}

	void MarkCarryingService(int passenger_id, int node_type, int ServiceTime)
	{
		if (node_type == 1)
			StartCarryingService(passenger_id, ServiceTime);
		if (node_type == 2)
			CompleteCarryingService(passenger_id, ServiceTime);

	}

	bool IsAllServiceComplete()
	{
		if (passenger_carrying_state.size() == 0)
			return true;
		else
			return false;
	}

	void CalculateLabelCost(int vehicle_id,int curTime)
	{
		LabelCost = 0;
		PrimalLabelCost = 0;
		TotalDeliveryTime = 0;
		TotalDeliveried = 0;
		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			int passenger_id = it->first;
			if (it->second == 2)  // complete
			{

				TotalDeliveried++;
				//LabelCost -= local_vehicle_passenger_additional_profit[vehicle_id][passenger_id];

				TotalDeliveryTime += max(0, passenger_service_time[passenger_id] - g_passenger_place_time[passenger_id]);
				LabelCost -= g_passenger_base_profit[passenger_id];
				LabelCost += max(0, passenger_service_time[passenger_id] - g_passenger_order_time[passenger_id]);
				PrimalLabelCost += max(0, passenger_service_time[passenger_id] - g_passenger_order_time[passenger_id]);
			}
			else if (it->second == 1) //not delivery
			{
				if (m_bRoute_close)
				{
					LabelCost += _PENALTY_OF_NOT_DELIVERY;
					PrimalLabelCost += _PENALTY_OF_NOT_DELIVERY;
				}
				else
				{
					int deltaTime = curTime - g_passenger_min_delivery_time[passenger_id];
					if (deltaTime > 0 && deltaTime < 20)
					{
						LabelCost += deltaTime * 3;
						PrimalLabelCost += deltaTime * 3;
					}
					else if (deltaTime >= 20)
					{
						LabelCost += deltaTime * 100;
						PrimalLabelCost += deltaTime * 100;
					}
				}
			}
		}
	}
	int GetLastServiceTime()
	{
		int maxsvcTime = 0;
		for (std::map<int, int>::iterator it = passenger_service_time.begin(); it != passenger_service_time.end(); ++it)
		{
			if (it->second > maxsvcTime)
				maxsvcTime = it->second;
		}
		return maxsvcTime;
	}
	int GetTotalTT()
	{
		if (m_visit_sequence.size() == 0)
			return 0;
		//int first_node_id = m_visit_sequence[0];
		//int deport_departure_time = 0;
		//if (g_node_type[first_node_id] == 1)
		//{
		//	int passenger_id = g_node_passenger_id[first_node_id];
		//	deport_departure_time = this->passenger_pickup_time[passenger_id];
		//}
		int lastT = GetLastServiceTime();
		return lastT ;
	}
	void WriteToFile(FILE* poutf)
	{
		stringstream s;

		for (int i = 0; i < m_visit_sequence.size(); i++)
		{
			int node_id = m_visit_sequence[i];
			int pax_id = g_node_passenger_id[node_id];
			int nt = g_node_type[node_id];
			int time = 0;
			if (nt == 1)
			{
				time = passenger_pickup_time[pax_id];
				s << "pax=, " << pax_id << ", pick up time=, " << time << endl;

			}
			else
			{
				time = passenger_service_time[pax_id];
				s << "pax=, " << pax_id << ", delivery time=,  " << time << endl;
			}
		}
		string converted(s.str());

		fprintf(poutf, converted.c_str());

	}
	void CountPassengerNumberOfVisits(int vehicle_id)
	{

		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			//if (it->second == 2)  // complete
			{
				int passenger_id = it->first;
#pragma omp critical
				{
					g_passenger_number_of_visits[passenger_id] +=1;
					g_passenger_vehicle_visit_flag[passenger_id][vehicle_id] = 1;
				}

			}
		}

	}
	std::string generate_string_key()
	{
		//std::string string_key;
		stringstream s;

		s << "n";
		s << current_node_id;  // space key
		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			s << "_";

			s << it->first << "["<< it->second << "]";

		}
		string converted(s.str());
		return converted;
		//string_key += converted;
		//return string_key;  //e.g. n4_1[1]_3[2]_4[2]
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

	void update_state(CVSState new_element)
	{
		std::string string_key = new_element.generate_string_key();
		int state_index = m_find_state_index(string_key);
		if (state_index == -1)  // no such state at this time
		{
			// add new state
			state_index = m_VSStateVector.size();
			m_VSStateVector.push_back(new_element);
			m_state_map[string_key] = state_index;
		}
		else
		{
			if (new_element.LabelCost < m_VSStateVector[state_index].LabelCost)
			{
				m_VSStateVector[state_index].Copy(&new_element);
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
		if(m_VSStateVector.size() > 2* BestKValue)
		{
			std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

			m_state_map.clear(); // invalid
			m_VSStateVector.erase(m_VSStateVector.begin()+ BestKValue, m_VSStateVector.end());
		}
	}

	float GetBestValue(int DualPriceFlag, int vehicle_id)
	{
		std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

		if (m_VSStateVector.size() >= 1)
		{
			std::string state_str = m_VSStateVector[0].generate_string_key();
			m_VSStateVector[0].CountPassengerNumberOfVisits(vehicle_id);
			if (DualPriceFlag == 1)
			{
				fprintf(g_pFileDebugLog, "Dual \t{{%s}}; Label Cost %f\n",
					state_str.c_str(), m_VSStateVector[0].LabelCost);
			}
			else
			{
				fprintf(g_pFileDebugLog, "Primal \t{{%s}}; Label Cost %f\n",
					state_str.c_str(), m_VSStateVector[0].PrimalLabelCost);
				//fprintf(g_pFileDebugLog, "Primal \t{{%s}}; Label Cost %f\n",
				//	state_str.c_str(), m_VSStateVector[0].LabelCost);
			}


			if(DualPriceFlag ==1)
				return m_VSStateVector[0].LabelCost;
			else
				return m_VSStateVector[0].PrimalLabelCost;
				//return m_VSStateVector[0].LabelCost;
		}
		else
			return _MAX_LABEL_COST;
	}

	CVSState* GetFirstVSState()
	{
		if (m_VSStateVector.size() == 0)
			return NULL;
		return &m_VSStateVector[0];
	}


};


C_time_indexed_state_vector g_time_dependent_state_vector[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];

C_time_indexed_state_vector g_ending_state_vector[_MAX_NUMBER_OF_VEHICLES];  // for collecting the final feasible states accesible to the depot

vector<CVSState> g_best_solution;


int g_node_status_array;
float g_node_label_cost;


// for non service link: one element: w2 = w1 for all possible stages
// for pick up link: one element: w2= w1 + the passenger of upstream node, p

// infeasible, p already in w1 or w1 is full at capacity

// for delivery link: one element: w2= w1 - the passenger of downstream node
//infeasible: if p is inc

//sequential computing version

//parallel computing version
float g_optimal_time_dependenet_dynamic_programming(
	int vehicle_id,
	int origin_node,
	int departure_time,
	int destination_node,
	int arrival_time,
	int vehicle_capacity,
	int BestKSize,
	int DualCostFlag )
	// time-dependent label correcting algorithm with double queue implementation
{

	if (arrival_time > g_number_of_time_intervals || g_outbound_node_size[origin_node] == 0)
	{
		return _MAX_LABEL_COST;
	}

	for (int p = 1; p < g_number_of_passengers; p++)
	{
		g_passenger_vehicle_visit_flag[p][vehicle_id] = 0;
	}
	//step 2: Initialization for origin node at the preferred departure time, at departure time
	for (int t = departure_time; t <= arrival_time; t++)  //first loop: time
	{
		g_time_dependent_state_vector[vehicle_id][t].Reset();
	}
	g_ending_state_vector[vehicle_id].Reset();
	CVSState element;
	element.current_node_id = origin_node;
	g_time_dependent_state_vector[vehicle_id][departure_time].update_state(element);

	// step 3: //dynamic programming
	for (int t = departure_time; t <= arrival_time; t++)  //first loop: time
	{
		// step 1: sort
		g_time_dependent_state_vector[vehicle_id][t].Sort();
		// step 2: scan the best k elements
		for (int w_index = 0; w_index < min(BestKSize, g_time_dependent_state_vector[vehicle_id][t].m_VSStateVector.size()); w_index++)
		{
			CVSState* pElement = &(g_time_dependent_state_vector[vehicle_id][t].m_VSStateVector[w_index]);

			int from_node = pElement->current_node_id;
			// step 2.1 from node to to node
			for (int i = 0; i < g_outbound_node_size[from_node]; i++)
			{
				int to_node = g_outbound_node_id[from_node][i];
				int to_node_passenger_id = g_node_passenger_id[to_node];
				int to_node_type = g_node_type[to_node];
				int link_no = g_outbound_link_no[from_node][i];

				int next_time = max(g_node_timestamp[to_node], t + g_link_free_flow_travel_time[link_no]);

				// step 2.2. check feasibility of node type with the current element
				if (next_time <= arrival_time )
				{
					if (to_node_passenger_id > 0 &&
						g_vehicle_passenger_visit_allowed_flag[vehicle_id][to_node_passenger_id] == 1)
					{

						if ((to_node_type == 1 && pElement->GetPassengerServiceState(to_node_passenger_id) == 0)
							|| (to_node_type == 2 && pElement->GetPassengerServiceState(to_node_passenger_id) == 1))  // pick up)
						{
							// step 2.3 label cost updating inside this function

							CVSState new_element;
							new_element.Copy(pElement);
							new_element.MarkCarryingService(to_node_passenger_id, to_node_type, next_time);

							new_element.current_node_id = to_node;
							new_element.passenger_service_state[to_node_passenger_id] = to_node_type;
							new_element.m_visit_sequence.push_back(to_node);
							new_element.CalculateLabelCost(vehicle_id, next_time);

							int link_no = g_outbound_link_no[from_node][i];

							if (to_node == destination_node && new_element.IsAllServiceComplete())
							{
								new_element.m_bRoute_close = true;
								new_element.CalculateLabelCost(vehicle_id, next_time);
								g_ending_state_vector[vehicle_id].update_state(new_element);
								g_ending_state_vector[vehicle_id].SortAndCleanEndingState(BestKSize);
							}

							g_time_dependent_state_vector[vehicle_id][next_time].update_state(new_element);
						}
					}
					else if (to_node == destination_node)
					{
						pElement->m_bRoute_close = true;
						pElement->CalculateLabelCost(vehicle_id, next_time);
						g_ending_state_vector[vehicle_id].update_state(*pElement);
						g_ending_state_vector[vehicle_id].SortAndCleanEndingState(BestKSize);
					}
				}
			}

		}
		} // for all time t


	// no backtrace
	return g_ending_state_vector[vehicle_id].GetBestValue(DualCostFlag, vehicle_id);
}

void g_ReadInputData()
{

	// initialization
	for (int i = 0; i < _MAX_NUMBER_OF_NODES; i++)
	{
		g_outbound_node_size[i] = 0;
		g_inbound_node_size[i] = 0;
	}

	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0

	int interval_node_no = 1;
	// step 1: read node file
	CCSVParser parser;
	for (int i = 0; i < _MAX_NUMBER_OF_PASSENGERS;i++)
		g_passenger_place_time[i] = -1;
	if (parser.OpenCSVFile("input_node.csv", true))
	{
		std::map<int, int> node_id_map;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string name;

			int node_type = 0;
			int node_id;
			int timestamp = 0;
			float baseprofit = 0;
			int passenger_id = -1;
			double X;
			double Y;
			int placetime;
			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;

			if (node_id <= 0 || g_number_of_nodes >= _MAX_NUMBER_OF_NODES)
			{
				cout << "node_id " << node_id << " is out of range" << endl;
				g_ProgramStop();
			}
			parser.GetValueByFieldName("node_type", node_type);
			parser.GetValueByFieldName("passenger_id", passenger_id);

			parser.GetValueByFieldName("timestamp", timestamp);
			parser.GetValueByFieldName("baseprofit", baseprofit);
			parser.GetValueByFieldName("placetime", placetime);

			g_node_type[node_id] = node_type;  //key nodeid; 1: pick up, 2: drop off
			g_node_passenger_id[node_id] = passenger_id;  //key nodeid; 1: pick up, 2: drop off
			g_node_timestamp[node_id] = timestamp;  //key nodeid;, values type 1: pick up:ready time, 2: order time
			g_node_baseprofit[node_id] = baseprofit;

			if(passenger_id>0)
			{
				if(baseprofit > 0)
					g_passenger_base_profit[passenger_id] = baseprofit;
				if (node_type == 1)
					g_passenger_order_time[passenger_id] = timestamp;
				else if (node_type == 2)
					g_passenger_min_delivery_time[passenger_id] = timestamp;
				if(placetime >= 0 && g_passenger_place_time[passenger_id] < 0)
					g_passenger_place_time[passenger_id] = placetime;
			}
			if (passenger_id > g_number_of_passengers)
			{
				g_number_of_passengers = passenger_id;
			}

			parser.GetValueByFieldName("x", X);
			parser.GetValueByFieldName("y", Y);

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 ==0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}

		cout << "number of nodes = " << g_number_of_nodes << endl;

		g_number_of_physical_nodes = g_number_of_nodes;

		fprintf(g_pFileOutputLog, "number of nodes =,%d\n", g_number_of_nodes);
		parser.CloseCSVFile();
	}

	// step 2: read link file

	if (parser.OpenCSVFile("input_link.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			if (parser.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;

			if (from_node_id <= 0 )
			{
				cout << "from_node_id " << from_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (to_node_id <= 0 )
			{
				cout << "to_node_id " << to_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			// add the to node id into the outbound (adjacent) node list

			int direction = 1;
			parser.GetValueByFieldName("direction", direction);

			if (direction <= -2 || direction >= 2)
			{
				cout << "direction " << direction << " is out of range" << endl;
				g_ProgramStop();
			}



				g_outbound_node_id[from_node_id][g_outbound_node_size[from_node_id]] = to_node_id;
				g_outbound_link_no[from_node_id][g_outbound_node_size[from_node_id]] = g_number_of_links;
				g_outbound_node_size[from_node_id]++;

				g_inbound_node_id[to_node_id][g_inbound_node_size[to_node_id]] = from_node_id;
				g_inbound_link_no[to_node_id][g_inbound_node_size[to_node_id]] = g_number_of_links;
				g_inbound_node_size[to_node_id]++;

				float link_length = 1;
				int number_of_lanes = 1;
				float travel_time = 1.0;

				parser.GetValueByFieldName("travel_time", travel_time);

				if (travel_time > 100)
				{
					cout << "travel_time > 100";
					g_ProgramStop();
				}

				g_link_free_flow_travel_time[g_number_of_links] = max(1, travel_time +0.5);   // at least 1 min, round to nearest integers
				g_link_free_flow_travel_time_float_value[g_number_of_links] = travel_time;


				// increase the link counter by 1
				g_number_of_links++;

				if (g_number_of_links % 1000==0)
					cout << "reading " << g_number_of_links << " links.. " << endl;
		}

		cout << "number of links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);

		parser.CloseCSVFile();
	}

	if (parser.OpenCSVFile("input_agent.csv", true))
	{
		V2PAssignment element;
		g_VRP_data.V2PAssignmentVector.push_back(element);

		CVSState vsta;
		g_best_solution.push_back(vsta);

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			int agent_id;
			parser.GetValueByFieldName("agent_id", agent_id);

			int agent_type = 0;
			parser.GetValueByFieldName("agent_type", agent_type);
			int from_node_id;
			int to_node_id;

			  // vehicle


				int vehicle_no = agent_id;

				if (agent_type == 1)
				{

				parser.GetValueByFieldName("from_node_id", from_node_id);
				parser.GetValueByFieldName("to_node_id", to_node_id);

				g_vehicle_origin_node[vehicle_no] = from_node_id;
				g_vehicle_destination_node[vehicle_no] = to_node_id;

				parser.GetValueByFieldName("departure_time", g_vehicle_departure_time[vehicle_no]);
				int departure_time_window = 0;
				parser.GetValueByFieldName("arrival_time", g_vehicle_arrival_time[vehicle_no]);

				if (g_vehicle_arrival_time[vehicle_no] < 0)
				{
					cout << "Vehicle data must have values in field arrival_time_start in file input_agent.csv!" << endl;
					g_ProgramStop();
				}

				g_vehicle_arrival_time[vehicle_no] = g_vehicle_arrival_time[vehicle_no];

				g_number_of_time_intervals = max(g_vehicle_arrival_time[vehicle_no] + 10, g_number_of_time_intervals);

				g_vehicle_capacity[vehicle_no] = -1;

				int capacity =1;
				parser.GetValueByFieldName("capacity", capacity);
				g_vehicle_capacity[vehicle_no] = max(1, capacity);
				if (g_vehicle_capacity[vehicle_no] < 0)
				{
					cout << "Vehicle data must have values in field capacity in file input_agent.csv!" << endl;
					g_ProgramStop();
				}
				parser.GetValueByFieldName("VOIVTT_per_hour", g_VOIVTT_per_hour[vehicle_no]);
				parser.GetValueByFieldName("VOWT_per_hour", g_VOWT_per_hour[vehicle_no]);

				if (g_max_vehicle_capacity < g_vehicle_capacity[vehicle_no])
					g_max_vehicle_capacity = g_vehicle_capacity[vehicle_no];

				}
				g_number_of_vehicles++;

				g_best_solution.push_back(vsta);

		}
		parser.CloseCSVFile();
	}
	for (int i = 0; i < g_number_of_passengers; i++)
	{
		V2PAssignment element;
		g_VRP_data.V2PAssignmentVector.push_back(element);
	}

	fprintf(g_pFileOutputLog, "number of vehicles =,%d\n", g_number_of_vehicles);

	cout << "read " << g_number_of_nodes << " nodes, " << g_number_of_links << " links" << ", " << g_number_of_passengers << " passengers, " << g_number_of_vehicles << "vehicles" << endl;
	fprintf(g_pFileDebugLog, "Network has %d nodes, %d links, %d  passengers, %d vehicles\n\n",
		g_number_of_nodes, g_number_of_links, g_number_of_passengers, g_number_of_vehicles);

	for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
	{
		int from_node = g_link_from_node_number[link];

		int to_node = g_link_to_node_number[link];

		if (g_link_service_code[link] != 0)
		{

		fprintf(g_pFileDebugLog, "link no.%d,  %d->%d, service code %d\n",
			link + 1,
			from_node,
			to_node,
			g_link_service_code[link]);
		}
	}

	fprintf(g_pFileDebugLog, "\n");

}

bool g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables(VRP_exchange_data* local_vrp_data)  // with varaible y only
{


	//cout << "Start scheduling passengers by Lagrangian Relaxation method" << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;
	g_SolutionStartTime = CTime::GetCurrentTime();


	//loop for each LR iteration

	float g_best_upper_bound = 99999;
	float g_best_lower_bound = -99999;


	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{

		local_vrp_data->reset_output();

		// reset the vertex visit count
		double LR_global_lower_bound = 0;


		//cout << "Lagrangian Iteration " << LR_iteration << "/" << g_number_of_LR_iterations << endl;
		//fprintf(g_pFileDebugLog, "---------------------------BBNodeNo %d-Lagrangian Iteration: %d ---------------------------", local_vrp_data->BBNodeNo, LR_iteration + 1);

		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		double TotalWaitingTimeCost = 0;

		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_passenger_number_of_visits[p] = 0;
		}

		for (int v = 1; v <= g_number_of_vehicles; v++)
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				g_vehicle_passenger_visit_allowed_flag[v][p] = 1;
				local_vehicle_passenger_additional_profit[v][p] = 0;
			}

			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				int assigned_vehicle_id = local_vrp_data->V2PAssignmentVector[p].input_assigned_vehicle_id;
				if (assigned_vehicle_id > 1)
				{
					local_vehicle_passenger_additional_profit[assigned_vehicle_id][p] = 1000;
					// assign very large profit to attract assigned vehicle
				}

				for (int vi = 0; vi < local_vrp_data->V2PAssignmentVector[p].input_prohibited_vehicle_id_vector.size(); vi++)
				{  // scan through the local VRP data, prohibit the input vehicles
					int vehicle_id = local_vrp_data->V2PAssignmentVector[p].input_prohibited_vehicle_id_vector[vi];
					g_vehicle_passenger_visit_allowed_flag[vehicle_id][p] = 0;
				}
			}


			fprintf(g_pFileDebugLog, "---------------------------BBNodeNo %d-Lagrangian Iteration: %d\n ---------------------------", local_vrp_data->BBNodeNo, LR_iteration + 1);

		// set arc cost, to_node_cost and waiting_cost for vehicles
//#pragma omp parallel for
			for (int v = 1; v <= g_number_of_vehicles; v++)
		{
				fprintf(g_pFileDebugLog,
					"\Debug: LB iteration %d, Vehicle %d performing DP: origin %d -> destination %d\n ",
					LR_iteration,
					v,
					g_vehicle_origin_node[v],
					g_vehicle_destination_node[v])	;


				float path_cost_by_vehicle_v =
					g_optimal_time_dependenet_dynamic_programming
					(
					v,
					g_vehicle_origin_node[v],
					g_vehicle_departure_time[v],
					g_vehicle_destination_node[v],
					g_vehicle_arrival_time[v],
					g_vehicle_capacity[v],
					10,
					1);
				if(path_cost_by_vehicle_v < _MAX_LABEL_COST)
					LR_global_lower_bound += path_cost_by_vehicle_v;

				fprintf(g_pFileDebugLog, "vehicle %d LR_global_lower_bound %f += path_cost_by_vehicle_v %f \n", v, LR_global_lower_bound,path_cost_by_vehicle_v);

		}  //for each v
		   //min CX + lamda(1 - # of visits)
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				LR_global_lower_bound += g_passenger_base_profit[p];
			}

		//	 step 3: scan all vehicles to mark the useage of the corresponding vertex

			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				float StepSize = 1/ (LR_iteration + 1.0f);
				if (StepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
				{
					StepSize = g_minimum_subgradient_step_size;
				}

				int constant = 5;
				g_passenger_base_profit[p]+= constant*StepSize * (g_passenger_number_of_visits[p] - 1);  // decrease the value and create profit
			}

			g_best_lower_bound = max(g_best_lower_bound, LR_global_lower_bound);  // keep the best lower bound till current iteration

			// at last LR iteration
			if (LR_iteration == g_number_of_LR_iterations - 1)  // last iteration
			{
				for (int p = 1; p <= g_number_of_passengers; p++)
				{
					if (g_passenger_number_of_visits[p] >= 2)
					{
						for (int v = 1; v <= g_number_of_vehicles; v++)
						{
							if (g_passenger_vehicle_visit_flag[p][v] == 1)
							{
								local_vrp_data->V2PAssignmentVector[p].AddCompettingVehID(v);
							}
						}
					}
				}
			}

		}

	// end of LR iterations

	// generate upper bound
	fprintf(g_pFileDebugLog, "\nGenerate upper bound\n");

	double LR_global_upper_bound = 0;
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		g_passenger_number_of_visits[p] = 0;
	}


	for (int v = 1; v <= g_number_of_vehicles; v++)
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_vehicle_passenger_visit_allowed_flag[v][p] = 1;
		}

	// set arc cost, to_node_cost and waiting_cost for vehicles
	// sequential DP for each vehicle, based on the LR prices
	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		float path_cost_by_vehicle_v =
			g_optimal_time_dependenet_dynamic_programming
			(
				v,
				g_vehicle_origin_node[v],
				g_vehicle_departure_time[v],
				g_vehicle_destination_node[v],
				g_vehicle_arrival_time[v],
				g_vehicle_capacity[v],
				10,
				0);
		if (path_cost_by_vehicle_v < _MAX_LABEL_COST)
			LR_global_upper_bound += path_cost_by_vehicle_v;
		fprintf(g_pFileDebugLog, "Vehicle %d LR_global_upper_bound %f += path_cost_by_vehicle_v %f\n",v, LR_global_upper_bound, path_cost_by_vehicle_v );

		g_best_upper_bound = min(g_best_upper_bound, LR_global_upper_bound);  // keep the best lower bound till current iteration

		//
		if (v < g_number_of_vehicles)  // mark the passsengers have been visited
		{
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				if (g_passenger_number_of_visits[p] >= 1)
				{
					g_vehicle_passenger_visit_allowed_flag[v + 1][p] = 0;  // not allowed to visit
					//fprintf(g_pFileDebugLog, "\nupper bound generation, for vehicle %d, pax %d is not allowed or no needed to serve",
					//	v+1,p);

				}
			}
		}
	}  //end of vehicle v
	//penalty for passenger who not was serviced
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		if (g_passenger_number_of_visits[p] < 1)
			g_best_upper_bound += g_passenger_base_profit[p];
	}

		CTimeSpan ctime = CTime::GetCurrentTime() - g_SolutionStartTime;
				cout << "\nComputational time:," << ctime.GetTotalSeconds() << endl;


		//fprintf(g_pFileDebugLog, "Summary: Iteration %d: Lower Bound = %.2f, upper Bound = %.2f, gap = %.2f, relative_gap = %.2f%%, # of pax not served = %d\n",

		//	g_best_lower_bound,
		//	g_best_upper_bound,
		//	(g_best_upper_bound - g_best_lower_bound),
		//	(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0
		//
		//	);
		//fprintf(g_pFileDebugLog, "******************************************************************************************************************\n\n");
		//fprintf(g_pFileOutputLog, "%d,%d,%f,%f,%f,%.3f%%,",
		//	local_vrp_data->BBNodeNo,
		//	LR_iteration+1,
		//	g_best_lower_bound,
		//	g_best_upper_bound,
		//	(g_best_upper_bound - g_best_lower_bound),
		//	(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0
		//	);
		//}
		cout << "End of Lagrangian Iteration Process " << endl;

	local_vrp_data->LBCost = g_best_lower_bound;
	local_vrp_data->UBCost = g_best_upper_bound;

	fprintf(g_pFileDebugLog, "local_vrp_data->LBCost=%f,local_vrp_data->UBCost=%f\n", local_vrp_data->LBCost, local_vrp_data->UBCost);


	return true;
}
void g_save_best_solution()
{

	for (int v = 1;v <= g_number_of_vehicles; v++)
	{
		CVSState* pstate = g_ending_state_vector[v].GetFirstVSState();
		if (pstate == NULL)
			g_best_solution[v].Clear();
		else
			g_best_solution[v].Copy(pstate);
	}

}
void g_write_solution()
{
	float totaltime = 0;
	int totalpassenger = 0;
	fprintf(g_pFileOutputLog, "Solution Outputs\n");
	char cbuf[100];
	int tot_routeTime = 0;
	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		if (g_best_solution[v].IsEmpty())
			continue;
		fprintf(g_pFileOutputLog, "Vehicle %d Route:\n",v);
		g_best_solution[v].WriteToFile(g_pFileOutputLog);

		totaltime += g_best_solution[v].TotalDeliveryTime;
		totalpassenger += g_best_solution[v].TotalDeliveried;
		tot_routeTime += g_best_solution[v].GetTotalTT();
	}
	float ra = (float)totalpassenger / (float)(tot_routeTime/60.0f);

	fprintf(g_pFileOutputLog, "Average deliveries/hour = %6.3f\n", ra);
	ra = totaltime / totalpassenger;
	fprintf(g_pFileOutputLog, "Average duration = %6.3f\n", ra);
	ra = totalpassenger;
	fprintf(g_pFileOutputLog, "Total Deliveries = %6.3f\n", ra);

}
void g_ReadConfiguration()
{
	CCSVParser parser;
	if (parser.OpenCSVFile("input_configuration.csv", true))
	{

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			string name;

			int node_type;
			int node_id;
			int number_of_threads = 1;
			double X;
			double Y;
			parser.GetValueByFieldName("number_of_iterations", g_number_of_LR_iterations);
			parser.GetValueByFieldName("shortest_path_debugging_details", g_shortest_path_debugging_flag);
			parser.GetValueByFieldName("dummy_vehicle_cost_per_hour", g_dummy_vehicle_cost_per_hour);
			parser.GetValueByFieldName("max_number_of_threads_to_be_used", number_of_threads);
			break;  // only the first line
		}
		parser.CloseCSVFile();
	}
}


int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
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
	 g_pFileAgentPathLog = fopen("agent_path.csv", "w");
	 if (g_pFileAgentPathLog == NULL)
	 {
		 cout << "File agent_path.csv cannot be opened." << endl;
		 g_ProgramStop();
	 }

	 fprintf(g_pFileAgentPathLog, "iteration_no,agent_id,agent_type,virtual_vehicle,path_node_sequence,path_time_sequence,path_state_sequence,\n"); // header
	 g_ReadInputData();
	// definte timestamps
	clock_t start_t, end_t, total_t;


	start_t = clock();

	//g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables(&g_VRP_data);

	g_Brand_and_Bound();

	g_write_solution();

	end_t = clock();
	total_t = (end_t - start_t);

	cout << "CPU Running Time = " << total_t << " milliseconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %ld milliseconds\n", total_t);
	fprintf(g_pFileOutputLog, "CPU Running Time =,%ld, milliseconds\n", total_t);

	fclose(g_pFileOutputLog);
	fclose(g_pFileDebugLog);
	fclose(g_pFileAgentPathLog);

	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;

	return 1;
}
