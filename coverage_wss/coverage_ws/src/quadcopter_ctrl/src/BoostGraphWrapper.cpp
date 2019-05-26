#include <BoostGraphWrapper.h>

BoostGraphWrapper::BoostGraphWrapper(std::string file_path)
{
    std::ifstream access_matrix;
    access_matrix.open(file_path.c_str());
    if (!access_matrix.is_open())
    {
        printf("%sAccess matrix not found! (sure is the executable folder?)%s\n", TC_RED, TC_NONE);
        exit(EXIT_FAILURE);
    }
    
    loadMatrixFile(access_matrix);
    graph_ptr = new Graph(gridSizeX * gridSizeY);
    index = get(vertex_index2, *graph_ptr);
    distances = get(edge_weight, *graph_ptr);
	
	int n = gridSizeX * gridSizeY;
	int row, col;    
	int row_shift, col_shift;
	int nb_row, nb_col;   
	int weight = 1;
	
	//std::cout << "Building Cycle " << std::endl;
	for (int i = 0; i < n; i++)
	{
		if (access_vec[i] == 0)
		{
			row = i / gridSizeY;
			col = i % gridSizeY;
			for (row_shift = -1; row_shift <= 1; row_shift++)
			{
				for (col_shift = -1; col_shift <= 1; col_shift++)
				{
					nb_row = row + row_shift;
					nb_col = col + col_shift;
					if ((nb_row >= 0) && (nb_row < gridSizeX) && (nb_col >= 0)
							&& (nb_col < gridSizeY) ///RANGE CHECK
							&& (row_shift != 0 || col_shift != 0) ///<--- don't check same node of current
							&& (row_shift * col_shift == 0) ///<--- don't allow diagonal movements
							)
					{
						if (access_vec[nb_row * gridSizeY + nb_col] == 0)
						{
							
							Vertex u;
							bool find_u = findVertex(i,u);
							if(!find_u)
							{
								u = add_vertex(*graph_ptr);
								index[u] = i;
							}
							
							Vertex v;
							bool find_v = findVertex(nb_row * gridSizeY + nb_col,v);
							if(!find_v)
							{
								v = add_vertex(*graph_ptr);
								index[v] = nb_row * gridSizeY + nb_col;
							}
							
							//std::cout << "Edge: source -> " << i << " " << "Edge: destination -> " << nb_row * gridSizeY + nb_col << std::endl;
							//std::cout << "Index: source -> " << index[u] << " " << "Index: destination -> " << index[v] << std::endl;
							
							Edge e;
							e = add_edge(u,v,*graph_ptr).first;
							distances[e] = weight;
							
							//std::cout << "Edge: source -> " << i << " " << "Edge: destination -> " << nb_row * gridSizeY + nb_col << std::endl;
						}
					}
				}
			}
		}
	}
	
	//std::cout << "Print Graph" << std::endl;
	//printGraph();
	//std::cout << "Print Nodes" << std::endl;
	//printVertexes();
}

void BoostGraphWrapper::printPath(std::vector<int> path)
{
	std::cout << "Computed Path" << std::endl;
	for(int i = 0; i < path.size(); i++)
	{
		std::cout << "Node: " << path[i] << std::endl;
	}
}

void BoostGraphWrapper::printVertexes()
{
	std::pair<vertex_iterator, vertex_iterator> vp = vertices(*graph_ptr);
	
	for (vertex_iterator vi = vp.first; vi != vp.second; ++vi)
	{	
		 std::cout << "Node: " << index[*vi] << std::endl;
	}
}

void BoostGraphWrapper::printGraph()
{	
	std::pair<edge_iterator, edge_iterator> ei = edges(*graph_ptr);
	
	for (edge_iterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter) 
	{
		Vertex s = source(*edge_iter, *graph_ptr);
		Vertex t = target(*edge_iter, *graph_ptr);
		int i = index[s];
		int j = index[t];
		std::cout << "Edge: source -> " << i << " " << "Edge: destination -> " << j << std::endl;
	}
}

bool BoostGraphWrapper::findVertex(int i, Vertex& vertex)
{
	std::pair<vertex_iterator, vertex_iterator> vp;
	for (vp = vertices(*graph_ptr); vp.first != vp.second; ++vp.first)
	{
		if(index[*vp.first] == i)
		{
			vertex = *vp.first;
			return true;
		}
		 //std::cout << index[*vp.first] << std::endl;
	}
	return false;
}

void BoostGraphWrapper::computeShortestPath(int i, int j, std::vector<int>& path)
{
	Vertex source;
	bool source_found = findVertex(i,source);
	
	Vertex destination;
	bool destination_found = findVertex(j,destination);
	
	if(source_found && destination_found)
	{
		path.push_back(j);
		
		std::vector<Vertex> p(num_vertices(*graph_ptr));
		std::vector<int> d(num_vertices(*graph_ptr));
		
		dijkstra_shortest_paths(*graph_ptr, source, predecessor_map(boost::make_iterator_property_map(p.begin(),get(boost::vertex_index, *graph_ptr))).distance_map(boost::make_iterator_property_map(d.begin(),get(boost::vertex_index, *graph_ptr))));
		 
		Vertex pred = p[destination]; 
		Edge e1;
		bool found;
		tie(e1, found) = edge(destination, pred, *graph_ptr);
		
		if (pred != destination) {
			while (pred != source) {
				int k = index[pred];
	
				path.insert(path.begin(), k);
				tie(e1, found) = edge(p[pred], pred, *graph_ptr);
				pred = p[pred];
			}
		}
	}
	else
	{
		std::cout << "Invalid source and destination nodes" << std::endl;
	}
}

BoostGraphWrapper::~BoostGraphWrapper()
{
}

std::string BoostGraphWrapper::get_selfpath()
{
    char buff[2048];
    ssize_t len = ::readlink("/proc/self/exe", buff, sizeof (buff) - 1);
    if (len != -1)
    {
        buff[len] = '\0';
        std::string path(buff); ///Here the executable name is still in
        std::string::size_type t = path.find_last_of("/"); 
        path = path.substr(0, t); 
        return path;
    }
    else
    {
        printf("Cannot determine file path!\n");
    }
}

void BoostGraphWrapper::loadMatrixFile(std::ifstream &access_mat)
{

	if (access_mat.is_open())
	{
		int val;
		int num_nl = 0;
		while (access_mat >> val)
		{
			if (access_mat.peek() == '\n')
				num_nl++;
			access_vec.push_back(val);
		}

		access_mat.close();

		gridSizeX = num_nl;
		gridSizeY = access_vec.size() / num_nl;

	}
	else
	{
		printf("Error reading file!\n");
	}
	
	/* Print access_vec
	std::cout << "Grid Size: " << gridSizeX << " x " << gridSizeY << std::endl;
	for(int i = 0; i < access_vec.size(); i++)
	{
		std::cout << access_vec[i] << std::endl;
	}
	*/

}
