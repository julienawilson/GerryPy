"""
Contains objects to pull tract information from database,
compute new congressional districts,
and store new information in a separate table.
"""
from gerrypy.models.mymodel import Tract, Edge
import networkx as nx


#  The tracts are ever tract
#  Edges represent every border between tracts.
#  Fill Graph makes a Network X graph with every tract as a node, and borders as edges.
def fill_graph(request):
    """Build global graph from tract and edge databases."""
    graph = nx.Graph()
    tracts = request.dbsession.query(Tract).all()  # get all tracts from db
    edges = request.dbsession.query(Edge).all()  # get all edges from db
    for tract in tracts:
        graph.add_node(tract)
    for edge in edges:
        source = request.dbsession.query(Tract).get(edge.tract_source)
        target = request.dbsession.query(Tract).get(edge.tract_target)
        graph.add_edge(source, target)
    return graph


class OccupiedDist(object):
    """A stucture to contain and separate tracts in a State object.

    add_node(self, node): adds node to nodes and updates district
    properties accordingly

    rem_node(self, node): removes node from nodes and updates district
    properties accordingly
    """

    def __init__(self, districtID, state_graph, tracts=None):
        """."""
        self.nodes = nx.Graph()
        self.perimeter = []
        self.population = 0
        self.area = 0
        self.districtID = districtID
        if tracts:
            try:
                for tract in tracts:
                    self.add_node(tract, state_graph)
            except TypeError:
                raise TypeError('Tracts must be iterable.')

    def add_node(self, node, state_graph):
        """Add node to nodes and updates district properties."""
        node.districtid = self.districtID
        self.nodes.add_node(node)
        for edge in state_graph.neighbors(node):  # (QUESTION) What does this do?
            if edge in self.nodes.nodes():
                self.nodes.add_edge(edge, node)
        self.population += node.tract_pop  # update the stats for the district
        self.area += node.shape_area
        if node in self.perimeter:  # Update the perimeter
            self.perimeter.remove(node)
        neighbors = state_graph.neighbors(node)
        for neighbor in neighbors:
            if neighbor not in self.nodes.nodes() and neighbor not in self.perimeter:
                self.perimeter.append(neighbor)

    def rem_node(self, node, state_graph):
        """Remove node from nodes and updates district properties."""
        self.population -= node.tract_pop
        self.nodes.remove_node(node)
        self.area -= node.shape_area 
        neighbors = state_graph.neighbors(node)
        to_perimeter = False
        for neighbor in neighbors:  # Decide whether to remove nodes from the perimeter.
            takeout = True
            if neighbor in self.perimeter:  # if its a perimeter node,
                neighborneighbors = state_graph.neighbors(neighbor)
                for neighborneighbor in neighborneighbors:  # check its neighbors
                    if neighborneighbor in self.nodes.nodes():  # if it has a neighbor in the district
                        takeout = False  # it should remain in the perimeter list.
                if takeout:  # If it should be removed,
                    self.perimeter.remove(neighbor)  # Remove it!
            elif neighbor in self.nodes.nodes():  # If the removed node neighbors the district (which it should)
                to_perimeter = True  # mark it to be added to the perimeter
        if to_perimeter:  # If its marked,
            self.perimeter.append(node)  # add it to the perimeter


# (QUESTION) Should we maybe just make this not inhereit anymore
class UnoccupiedDist(OccupiedDist):
    """A structure to contain tracts that haven't been claimed by a district.

    add_node(self, node): adds node to nodes and updates district
    properties accordingly

    rem_node(self, node): removes node from nodes and updates district
    properties accordingly
    """

    def __init__(self, districtID, state_graph, tracts=None):
        """."""  # (QUESTION) Fix this
        self.nodes = nx.Graph()  # (QUESTION) is this the best name for it? thats why we have to do nodes.nodes()
        self.perimeter = []
        self.population = 0
        self.area = 0
        self.districtID = districtID
        if tracts:
            try:
                for tract in tracts:
                    self.add_node(tract, state_graph)
            except TypeError:
                raise TypeError('Tracts must be iterable.')

    def add_node(self, node, state_graph):
        """Add node to nodes and updates district properties accordingly."""
        node.districtid = None
        self.nodes.add_node(node)
        for neighbor in state_graph.neighbors(node):  # After node is added, make the edge connections within the unoccupied district.
            if neighbor in self.nodes:
                self.nodes.add_edge(neighbor, node)
        self.population += node.tract_pop  # Update stats
        self.area += node.shape_area
        neighbors = state_graph.neighbors(node)
        to_add = False
        for neighbor in neighbors:
            takeout = True
            if neighbor in self.perimeter:  # (QUESTION) map this one out
                neighborneighbors = state_graph.neighbors(neighbor)
                for neighborneighbor in neighborneighbors:
                    if neighborneighbor not in self.nodes:
                        takeout = False
                if takeout:
                    self.perimeter.remove(neighbor)
            if neighbor not in self.nodes:
                to_add = True
        if to_add:
            self.perimeter.append(node)

    def rem_node(self, node, state_graph):
        """Remove node from nodes and updates district properties accordingly."""
        self.population -= node.tract_pop
        self.area -= node.shape_area
        if node in self.perimeter:
            self.perimeter.remove(node)
        neighbors = self.nodes.neighbors(node)  # (QUESTION) This whole thing could be a helper, right?
        for neighbor in neighbors:  # check the removed nodes neighbors
            if neighbor not in self.perimeter:  # if the neighbor is not in the perimeter, it should be. Nodes are removed from unoc when they are added to something else.
                self.perimeter.append(neighbor)  # add the node to the perimeter.
        self.nodes.remove_node(node)


class State(object):
    """Manages how tracts are distributed into districts in a particular state.
a
    build_district(self, start, population): (QUESTION) Should this be formatted differently?
    creates a new district stemming from the start node with a given population

    fill_state(self, request): continues to build districts until all unoccupied tracts are claimed (QUESTION) or until there are 7 districts?
    """

    def __init__(self, request, num_dst):
        """Build unoccupied district(s) for entire state."""
        self.unoccupied = []  # Unoccupied Districts: usually only 1
        self.districts = []  # Occupied districts
        self.population = 0
        self.area = 0
        self.num_dst = num_dst  # The Number of districts alotted for that state (7 for Colorado)
        self.state_graph = fill_graph(request)
        landmass = nx.connected_components(self.state_graph)  # Returns all of the connected/contiguous areas of land, for a state.
        for island in landmass:  # Island is set of nodes.
            unoc = UnoccupiedDist(None, self.state_graph, tracts=island)  # needs the state graph for its edges
            for tract in unoc.nodes.nodes():
                if tract.isborder == 1:  # this is a hardcoded field for colorado.  A challenge of adding more states is finding these automatically.
                    unoc.perimeter.append(tract)  # begin with all border tracts in the perimeter.
            self.population += unoc.population  # (QUESTION) Can a growing district grab from a different unoc??? e.g. if its a nearby island
            self.unoccupied.append(unoc)  # Each island or separate landmass gets added to the unoccupied district list.
            self.area += unoc.area
        self.target_pop = self.population // num_dst  # Average available population is the target population.  It could theorhetically drift high or low, but it hasn't been a problem.  It helps split the remaining popuation.

    def fill_state(self, request, criteria):
        """Build districts until all unoccupied tracts are claimed."""
        from gerrypy.graph_db_interact.assigndistrict import assign_district, populate_district_table

        for num in range(self.num_dst):  # Build the number of desired districts.  For each:
            rem_pop = 0
            for unoc in self.unoccupied:
                rem_pop += unoc.population  # Set the total remaining population not in a district.
            rem_dist = self.num_dst - len(self.districts)  # (QUESTION) This could also be '- num' yeah?
            tgt_population = rem_pop / rem_dist
            self.build_district(tgt_population, num + 1, criteria)
        assign_district(request, self.state_graph)  # Assigns the district ID to all tracts in the DB
        populate_district_table(request, self)  # Creates the DistrictView DB View for the finished districts. 
        if self.unoccupied:  # If there are remaining unclaimed tracts
            return False  # (QUESTION) This is not being used I dont think.
        return True

    def build_district(self, tgt_population, dist_num, criteria):
        """Create a new district stemming from the start node with a given population."""
        dst = OccupiedDist(dist_num, self.state_graph)
        self.districts.append(dst)
        start = self.find_start()  # Selects the first node for the District
        self.swap(dst, start)  # if state is full, this wont work (QUESTION) check on this
        while True:
            new_tract = self.select_next(dst, criteria)  # Selects the best choice for the next tract, given criteria
            if new_tract is None:  # (QUESTION) when is new_tract none?  No more nodes in unoc.
                for unoc in self.unoccupied:  # I think this triggers when we're done, it removes empty unoccupied districts.
                    if not len(unoc.nodes.nodes()):
                        self.unoccupied.remove(unoc)
                break  # If its empty, we stopp building.
            high_pop = (new_tract.tract_pop + dst.population)  # Population including the next tract.
            if abs(high_pop - tgt_population) > abs(dst.population - tgt_population):  # If the population including the next district is further from the goal,
                break  # we stop building.
            else:
                self.swap(dst, new_tract)  # Swap removes the tract from its unoccupied district and adds it to the occupied district.
                neighbors = self.state_graph.neighbors(new_tract)  # ok this part is tricky.
                unassigned_neighbors = [neighbor for neighbor in neighbors if neighbor in self.unoccupied[0].nodes]  # grab the new nodes unassigned neighbors
                if len(unassigned_neighbors) > 1:  # If there is more than one, than a split is possible. (QUESTION) why did we check this?
                    for i in range(len(unassigned_neighbors)):
                        if not nx.has_path(  # We check each node and its previous neighbor to ensure they're connected. (thanks, nx)
                            self.unoccupied[0].nodes,  # (QUESTION) IS there ever actually more than one thing in the unoc nodes
                            unassigned_neighbors[i],
                            unassigned_neighbors[i - 1]
                        ):  # If there is a split in the unoccupied district...
                            unoc_neighbors = [x for x in nx.connected_components(self.unoccupied[0].nodes)]  # Identify each of the distinct unoccupied districts.
                            biggest = max(unoc_neighbors, key=lambda x: len(x))
                            unoc_neighbors.remove(biggest)  # Ignore the largest
                            # all unoccupied districts will be bordering, because as soon as there is a split, we do this.
                            for neigh in unoc_neighbors:  # Consume all of the rest (usually one small one)
                                for tract in neigh:  # This sometimes gives us a district that is too large, and is a major focus of ours right now.
                                    self.swap(dst, tract)
                            break

    def swap(self, dst, new_tract):  # (QUESTION) removes a node from the unoccupied district, puts in in the dst
        """Exchange tract from unoccupied district to district."""
        # unoc_dst = None
        # for island in self.unoccupied:
        #     if new_tract in island.perimeter:
        #         unoc_dst = island  #(QUESTION) This is all we need?
        self.unoccupied[0].rem_node(new_tract, self.state_graph)
        dst.add_node(new_tract, self.state_graph)
        # return unoc_dst

    def select_next(self, dst, criteria):  # Select the next best node to join the District
        """Choose the next best tract to add to growing district."""
        best_rating = 0
        best = None  # We're going to build a score for each node based on our criteria.
        for perimeter_tract in dst.perimeter:  # dst.perimeter is every unassigned node bordering that district.
            if perimeter_tract.districtid is None:  # (QUESTION)  Was that a lie I just told? Does dst.permiter return all perm items?
                count = 0  # (QUESTION) we're assigning all these district ids, but does that not get wrapped in to DB?  we use a separeate function to do that OH nvm i get it.  one is on a graph, one is on the model.
                for neighbor in self.state_graph.neighbors(perimeter_tract):  # look at neighbors of each perimeter tract
                    if neighbor.districtid == dst.districtID:  # Check how many tracts that tract borders that are ALREADY in the district.  More borders gets more points
                        count += 1
                counties = set()  # County Score
                for node in dst.nodes:  # Find all the counties aleady in the district.
                    counties.add(node.county)
                same_county = 0  
                if perimeter_tract.county in counties:  # If the tracts county is in the district already, it gets a point.
                    same_county = 1
                rating = count * int(criteria['compactness']) + same_county * int(criteria['county'])  # Calculate score based on different criteria.  Recent version 
                if rating > best_rating:
                    best_rating = rating
                    best = perimeter_tract  # Save the best tract
        return best  # Return it!  If there is not perimeter, or no neighbors with no district Id, we return None.

    def find_start(self):
        """
        Choose best starting tract for a new district.
        Based on number of bordering districts.
        """
        best_set = set()
        best = None
        for tract in self.unoccupied[0].perimeter:  # Check nodes along unoc perimeter
            unique_dists = set()
            for neighbor in self.state_graph.neighbors(tract):  # Look at each of its neighbors
                for dst in self.districts:  # Record the number of districts it borders
                    if neighbor in dst.nodes.nodes():
                        unique_dists.add(dst)
            if len(unique_dists) > len(best_set) or len(unique_dists) == 0:  # grab the node if it borders the mot districts, or if its the firt one we looked at.
                best_set = unique_dists
                best = tract
        return best  # Return that selected node to start on.

    def split_unoccupied_dist(self, unoc_dst):  # (QUESTION) WE're not using this.  Delete it!
        """Remove unoccupied dist from State and adds contiguous unoccupied sub-districts."""
        self.unoccupied.remove(unoc_dst)
        index = len(self.unoccupied)
        landmass = nx.connected_components(unoc_dst.nodes)
        for island in landmass:
            self.unoccupied.append(UnoccupiedDist(None, self.state_graph, tracts=island))
        return self.unoccupied[index:]
