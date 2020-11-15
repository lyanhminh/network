import LightGraphs, GraphPlot, DataStructures, Random
import Base: push!, pop!, length

idx = 1

abstract type Node end

mutable struct UndirectedNode <: Node
    name
    neighbors::Dict{Node, Real}
    id::Int
    function UndirectedNode(name, neighbors)  
        global idx
        node = new(name, neighbors, idx)
        idx +=1 
        node
    end
end

mutable struct WeightedNode <: Node
    name
    neighbors::Dict{Node, Real}
    id::Int
    function WeightedNode(name, neighbors)  
        global idx
        node = new(name, neighbors, idx)
        idx +=1 
        node
    end
end
### NODE TYPE

UndirectedNode() = UndirectedNode(nothing, Dict() )
UndirectedNode(name) = UndirectedNode(name,Dict())
UndirectedNode(name, neighbors::Array{Node}) = UndirectedNode(name, Dict(node => 1 for node in neighbors))

WeightedNode() = WeightedNode(nothing, Dict() )
WeightedNode(name) = WeightedNode(name, Dict())
WeightedNode(name, neighbors::Dict{Node, Real}) = WeightedNode(name, neighbors)

function addNeighbors!(node::UndirectedNode, neighbors :: Node...)
    for neighbor in neighbors
        if !in(neighbor, keys(node.neighbors))
            node.neighbors[neighbor] = 1
            neighbor.neighbors[node] = 1
        end
    end
end

function addNeighbors!(node::WeightedNode, neighbors :: Dict{WeightedNode, <: Number})
    for (neighbor, weight) in neighbors
        node.neighbors[neighbor] = weight    
        neighbor.neighbors[node] = weight    
    end
end

function print(node :: Node, before, after)
        println("$(before)id: $(node.id), name: $(node.name)$(after)")  
end

function print(node :: Node, before)
        println("$(before)id: $(node.id), name: $(node.name)")  
end

function print(node::Node)
    println("id: $(node.id), name: $(node.name)")  
end

# function printNeighbors(node::UndirectedNode)
#     for neighbor in node.neighbors
#         print(neighbor, "neighbor: ")
#     end
# end

function printNeighbors(node::Node)
    for (neighbor, weight) in node.neighbors
        print(neighbor, "neighbor: ", ", weight: $(weight)")
    end
end

function isNeighbor(node::Node, target::Node)
    !isempty(filter(neighbor -> neighbor.id == target.id, collect(keys(node.neighbors)))) 
end

function setId!(node, id)
    node.id = id
    node
end


###   Frontier Class  -----------------------------------------------------------
abstract type Frontier end

struct BreadthFrontier <: Frontier 
    frontier::DataStructures.Queue{Node}
end

BreadthFrontier() = BreadthFrontier(DataStructures.Queue{Node}())

push!(frontier::BreadthFrontier, node::Node) =  DataStructures.enqueue!(frontier.frontier, node)
pop!(frontier::Frontier) = DataStructures.dequeue!(frontier.frontier)
length(frontier::Frontier) = length(frontier.frontier)
    
function addNeighborsToFrontier(frontier::Frontier, node::Node, explored::Dict)
    neighbors = collect(keys(node.neighbors))
    shuffledNeighbors = [neighbors[i] for i in Random.randperm(length(neighbors))]
    for neighbor in shuffledNeighbors
        if(!(neighbor.id in keys(explored))) 
            push!(frontier, neighbor) 
        end
    end
end

###   GRAPH TYPE  ------------------------------------------------------------
mutable struct Graph
    nodes::Dict{Int, Node}

end
Graph()= Graph(Dict())
Graph(nodes::Array{<:Node}) = Graph(Dict(node.id => node for node in nodes))

function connect!(G::Graph, outsideNodes::Node...)
    for outsideNode in outsideNodes
        G.nodes[outsideNode.id] = outsideNode
    end
end

function print(G::Graph)
    for (key, node) in G.nodes
        println("---------------------------\nnode:")
        print(node)
        println("\nneighbors:")
        printNeighbors(node)
    end
end

function searchPath(G::Graph, startNode::Node, target::Node, frontier::Frontier)
    explored = Dict()
    push!(frontier, startNode)
    
    function recurse(parent, tag, path)
        if length(frontier) == 0 
                return nothing
        end
        println("--------------------------\ntarget node: $(target.name)")
        node = pop!(frontier)
        
        neighbors =  node.neighbors
        for (child, weight) in neighbors
            if !(child in  keys(tag))
                tag[child] =  node
            end
        end
        path[node] = tag[node]
        
        println("this is node id $(node.id) with name $(node.name)")
        println("this is tag: ")
        for (key,thing) in tag
            println("node :($(key.name), \nparent: ")
            print(thing)
        end
        if isNeighbor(node, target)
            println("found target")
            path[target] = node
            return path
        end
        if !(node.id in keys(explored)) 
            explored[node.id] = node
            # add to neighbors to frontier
            addNeighborsToFrontier(frontier, node, explored)        
        end
        return recurse(node, tag,  path)
    end

    pathDict = recurse(startNode, Dict(startNode => startNode), Dict())
    return unnest(pathDict, target, startNode)
end

function asAdjacencyMatrix(G)
    nodes = collect(values(G.nodes))
    indexed = Dict(nodes[i] => i for i in 1:length(nodes))
    nodesDict = Dict(idxVal => nodeKey for (nodeKey, idxVal) in indexed)
    adjacenyArrays = []
    for (node, idx) in indexed
        neighborIndices = map(neighbor -> indexed[neighbor], collect(keys(node.neighbors)))
        adjacencyArray = [i in neighborIndices ? nodes[i].neighbors[node] : 0 for i in 1:length(nodes)]
        push!(adjacenyArrays, (idx, adjacencyArray))
    end
    A = reduce(hcat, map(x->x[2],sort(adjacenyArrays, by = x ->x[1])))
    names = [nodesDict[idx].name for idx in 1:size(A)[1]]
    [names, A]
end

function unnest(path,target, source) 
    function recurse(nodeList, previous)
        next = path[previous] 
        append!(nodeList, [next])
        if next == source
            return nodeList
        else
            recurse(nodeList, next)
        end
    end
   nodePath = reverse(recurse([target], target))

end

function showPath(path)  
    map(node -> node.name, path) 
end

function drawGraph(G, names)
    
    if  !(G isa Array)
        names, matrix = asAdjacencyMatrix(G)
        adjM = LightGraphs.Graph(matrix)
    else
        adjM = LightGraphs.Graph(G)
        matrix = G
    end
    println(names)
    println(adjM)
    edgeWeights = getEdgeWeights(matrix)
    GraphPlot.gplot(adjM, nodelabel = names,  edgelabel=edgeWeights, edgelabelc=GraphPlot.colorant"white")
end  

function getEdgeWeights(adM)
    weights = []
    d = size(adM)[1]
    for idx in 1:d
        append!(weights, adM[idx, idx:d])
    end
    filter(weight -> weight != 0, weights)
end

function drawPath(path, G)
    namesPath = showPath(path)
    names, matrix = asAdjacencyMatrix(G)
    nodecolor = [GraphPlot.colorant"orange", GraphPlot.colorant"lightseagreen"]
    membership = [x in namesPath ? 1 : 2 for x in names]
    nodeFill = nodecolor[membership]
    edgeWeights = getEdgeWeights(matrix)
    GraphPlot.gplot(LightGraphs.Graph(matrix), nodelabel=names, nodefillc=nodeFill, edgelabel=edgeWeights, edgelabelc=GraphPlot.colorant"white")
end


#   Testing Sandbox ------------------------------------------------------------------------------------


n, n2, n3, n4, n5, n6,n7, n8 = [UndirectedNode(i) for i in 1:8]
addNeighbors!(n,n2, n3)
addNeighbors!(n2, n5)
addNeighbors!(n, n4)
addNeighbors!(n4, n5)
addNeighbors!(n6, n3, n5)
addNeighbors!(n7, n3)
addNeighbors!(n, n7)
addNeighbors!(n2,n8)
G = Graph([n, n2, n3, n4, n5])
connect!(G, n7)
connect!(G, n8)
frontier = BreadthFrontier()
connect!(G, n6)
path = searchPath(G, n5, n3, frontier )
print(G)
startNode = n5
endNode = n3

dn, dn2, dn3, dn4, dn5, dn6, dn7, dn8, dn9, dn10, dn11, dn12, dn13, dn14, dn15 = [WeightedNode(i) for i in 1:15]
addNeighbors!(dn, Dict(dn2 => 1))
addNeighbors!(dn2, Dict(dn => 1, dn9 =>1, dn8 => 3, dn3=> 4))
addNeighbors!(dn3, Dict(dn5 => 2, dn4 => 4))
addNeighbors!(dn5, Dict(dn6 => 8, dn7 => 3, dn11 =>2))
addNeighbors!(dn6, Dict(dn10 => 1))
addNeighbors!(dn7, Dict(dn8 => 2, ))
addNeighbors!(dn8, Dict(dn11 => 1, dn12 =>1))
addNeighbors!(dn10, Dict(dn13 => 1, dn9 =>9))

DG = Graph([dn, dn2, dn3, dn4, dn5, dn6, dn7, dn8, dn9, dn10, dn11, dn12, dn13]) 




########################################### MAIN  ################################################
function main()

    n = Node(1)
    n2 = Node(2)
    n3 = Node(3)
    n4 = Node(4)
    n5 = Node(5)
    n6 = Node(6)
    n7 = Node(7)
    n8 = Node(8)
    addNeighbors!(n,n2, n3)
    addNeighbors!(n2, n5)
    addNeighbors!(n, n4)
    addNeighbors!(n4, n5)
    addNeighbors!(n6, n3, n5)
    addNeighbors!(n7, n3)
    addNeighbors!(n, n7)
    addNeighbors!(n2,n8)
    connect!(G, n7)
    connect!(G, n8)
    frontier = BreadthFrontier()
    G = Graph([n, n2, n3, n4, n5])
    connect!(G, n6)
    searchPath(G, n5, n3, frontier )
    
 end
 
 if abspath(PROGRAM_FILE) == @__FILE__
     main()
 end
