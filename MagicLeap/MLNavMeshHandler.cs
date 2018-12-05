using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;
using UnityEngine.Experimental.XR;
using Pathfinding;
using System;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class MLNavMeshHandler : MonoBehaviour {

    public MLSpatialMapper mapper;
    protected AstarPath pathing;
    private Dictionary<TrackableId, NavMeshGraph> idToNav = new Dictionary<TrackableId, NavMeshGraph>();
    private Dictionary<Int3, NavMeshGraph> cornerToGraph = new Dictionary<Int3, NavMeshGraph>();
    private Dictionary<NavMeshGraph, float> lastUpdate = new Dictionary<NavMeshGraph, float>();
    public float minimumUpdateInterval = 5; //the minimum seconds between processing updates to a particular mesh
    private int updateQueueMaxItems = 10;
    private bool updateInProgress = false;

    private Queue<NavMeshGraph> updateQueue = new Queue<NavMeshGraph>();

    private List<NavMeshGraph> toRemove = new List<NavMeshGraph>();
    public float maxStitchableGap = .1f;
    private float maxGap_Int3;
    private Vector3[] neighborDirections;


    Int3 v0, v1, v2;
    List<TriangleMeshNode> fromNodes;
    List<TriangleMeshNode> toNodes;
    bool isTouching;
    Vector3 center;

    public bool verbose;

    // Use this for initialization
    void Start() {
        maxGap_Int3 = Int3.PrecisionFactor * maxStitchableGap;

        if (mapper != null)
        {
            mapper.meshAdded += HandleOnMeshAdd;
            mapper.meshUpdated += HandleOnMeshUpdate;
            mapper.meshRemoved += HandleOnMeshRemove;
        }

        else
        {
            Debug.LogWarning("MLSpatialMapper not set. Disabling Script");
            this.enabled = false;
            return;
        }


        pathing = AstarPath.active;
        if(pathing == null)
        {
            Debug.LogWarning("Could not find active AstarPath. Disabling Script");
            this.enabled = false;
            return;
        }

        List<Vector3> directionsList = new List<Vector3>();
        for(int x = -1; x<=1; x++)
        {
            for(int y = -1; y<=1; y++)
            {
                for (int z = -1; z<=1; z++)
                {
                    if (x == 0 && y == 0 && z == 0) continue;
                    if ((x != 0 ? 1 : 0) + (y != 0 ? 1 : 0) + (z != 0 ? 1 : 0) > 2) continue;
                    directionsList.Add(new Vector3(x, y, z));
                }
            }
        }
        neighborDirections = directionsList.ToArray();
    }
    void OnDestroy()
    {
        if (mapper != null)
        {
            mapper.meshAdded -= HandleOnMeshAdd;
            mapper.meshUpdated -= HandleOnMeshUpdate;
            mapper.meshRemoved -= HandleOnMeshRemove;
        }
    }

    private void AddNavMesh(TrackableId id, MeshFilter filter)
    {
        if (verbose) Debug.Log("Add Mesh " + id);
        NavMeshGraph meshGraph = pathing.data.AddGraph((typeof(NavMeshGraph))) as NavMeshGraph;

        meshGraph.sourceMesh = filter.mesh;
        meshGraph.enableNavmeshCutting = false;
        meshGraph.recalculateNormals = false;

        idToNav.Add(id, meshGraph);

        if (filter.mesh.bounds.size != Vector3.zero)
        {

            Int3 corner = (Int3)GetCorner(meshGraph);


            if (!cornerToGraph.ContainsKey(corner))
            {
                cornerToGraph.Add(corner, meshGraph);
            }
            else
            {
                Debug.LogWarning("Corner " + corner + " already claimed");
            }

            QueueForUpdate(meshGraph);
        }
    }

    private void UpdateNavMesh(TrackableId id)
    {
        if (verbose) Debug.Log("Update Mesh " + id);

        NavMeshGraph graph;
        if (idToNav.TryGetValue(id, out graph))
        {
            QueueForUpdate(graph);
        }
    }

    private void RemoveNavGraph(TrackableId id)
    {
        if (verbose) Debug.Log("Remove Mesh " + id);

        NavMeshGraph graph;
        if (idToNav.TryGetValue(id, out graph))
        {
            toRemove.Add(graph);
            QueueForUpdate(graph);

            if (graph.sourceMesh.bounds.size != Vector3.zero)
            {
                cornerToGraph.Remove((Int3)GetCorner(graph));
            }
            if (lastUpdate.ContainsKey(graph)) lastUpdate.Remove(graph);

        }
        idToNav.Remove(id);
    }



    #region STITCHING
    private void StitchToNeighbors(NavMeshGraph myGraph)
    {
        Int3 corner = (Int3)GetCorner(myGraph);
        NavMeshGraph neighborGraph;
        foreach(Vector3 direction in neighborDirections)
        {
            if(cornerToGraph.TryGetValue(corner + (Int3)direction, out neighborGraph)){

               BuildNeighborMeshLinks(myGraph, neighborGraph, direction);

            }
        }
    }

    private void UnstitchFromNeighbors(NavMeshGraph myGraph)
    {        
        myGraph.GetNodes(node =>
        RemoveOffGraphConnections(node));
    }

    private int RemoveOffGraphConnections(GraphNode node)
    {
        List<GraphNode> offGraph = new List<GraphNode>();
        node.GetConnections(other => 
        {
            if (node.GraphIndex != other.GraphIndex)
            {
                offGraph.Add(other);
            }

        });
        foreach (GraphNode off in offGraph)
        {
            off.RemoveConnection(node);
            node.RemoveConnection(off);
        }
        return offGraph.Count;
    }


    private void BuildNeighborMeshLinks(NavMeshGraph from, NavMeshGraph to, Vector3 direction)
    {
        if(to == null)
        {
            Debug.LogError("To Null");
            return;
        }
        if(from == null)
        {
            Debug.LogError("From Null");
            return;
        }

        Int3 valueInt3 = 
            (Int3)(
                GetCorner(from) + new Vector3(
                Mathf.Max(0, direction.x),
                Mathf.Max(0, direction.y),
                Mathf.Max(0, direction.z))
            );

        //Debug.DrawLine(GetCorner(from) + Vector3.one * .5f, GetCorner(from) + Vector3.one * .5f + direction * .5f, Color.red, 10);

        fromNodes = new List<TriangleMeshNode>();
        toNodes = new List<TriangleMeshNode>();

        from.GetNodes(node => {
            if(NodeHas2VerticesTouchingEdge(node as TriangleMeshNode,direction, valueInt3)){
                fromNodes.Add(node as TriangleMeshNode);
            }
        });


        to.GetNodes(node => {
            if (NodeHas2VerticesTouchingEdge(node as TriangleMeshNode, direction, valueInt3))
            {
                toNodes.Add(node as TriangleMeshNode);
            }
        });

        uint cost = 1000;
        foreach (TriangleMeshNode fromNode in fromNodes)
        {
            foreach(TriangleMeshNode toNode in toNodes)
            {
                if(Vector3.Distance(fromNode.ClosestPointOnNode((Vector3)toNode.position), toNode.ClosestPointOnNode((Vector3)fromNode.position)) <= maxStitchableGap){
                    cost = (uint)(Int3.Precision * Vector3.Distance((Vector3)toNode.position, (Vector3)fromNode.position));
                    fromNode.AddConnection(toNode, cost);
                    toNode.AddConnection(fromNode, cost);
                }
            }
        }
    }

    private bool NodeHas2VerticesTouchingEdge(TriangleMeshNode node, Vector3 axisFilter, Int3 valueInt3)
    {

        isTouching = true;

        node.GetVertices(out v0, out v1, out v2);
        if(axisFilter.x != 0) isTouching = isTouching && (Mathf.Abs(v0.x - valueInt3.x) <= maxGap_Int3 ? 1 : 0) + (Mathf.Abs(v1.x - valueInt3.x) <= maxGap_Int3 ? 1 : 0) + (Mathf.Abs(v2.x - valueInt3.x) <= maxGap_Int3 ? 1 : 0) >= 1;
        if(axisFilter.y != 0) isTouching = isTouching && (Mathf.Abs(v0.y - valueInt3.y) <= maxGap_Int3 ? 1 : 0) + (Mathf.Abs(v1.y - valueInt3.y) <= maxGap_Int3 ? 1 : 0) + (Mathf.Abs(v2.y - valueInt3.y) <= maxGap_Int3 ? 1 : 0) >= 1;
        if(axisFilter.z != 0) isTouching = isTouching && (Mathf.Abs(v0.z - valueInt3.z) <= maxGap_Int3 ? 1 : 0) + (Mathf.Abs(v1.z - valueInt3.z) <= maxGap_Int3 ? 1 : 0) + (Mathf.Abs(v2.z - valueInt3.z) <= maxGap_Int3 ? 1 : 0) >= 1;

        return isTouching;
    }

    #endregion

    #region Event Handlers
    /// <summary>
    /// Handles the MeshReady event, which tracks and assigns the correct mesh renderer materials.
    /// </summary>
    /// <param name="meshId">Id of the mesh that got added / upated.</param>
    private void HandleOnMeshUpdate(TrackableId meshId)
    {
        if (mapper.meshIdToGameObjectMap.ContainsKey(meshId))
        {
            UpdateNavMesh(meshId);
        }

    }


    /// <summary>
    /// Handles the MeshReady event, which tracks and assigns the correct mesh renderer materials.
    /// </summary>
    /// <param name="meshId">Id of the mesh that got added / upated.</param>
    private void HandleOnMeshRemove(TrackableId meshId)
    {
        if (mapper.meshIdToGameObjectMap.ContainsKey(meshId))
        {
            RemoveNavGraph(meshId);
        }
    }


    /// <summary>
    /// Handles the MeshReady event, which tracks and assigns the correct mesh renderer materials.
    /// </summary>
    /// <param name="meshId">Id of the mesh that got added / upated.</param>
    private void HandleOnMeshAdd(TrackableId meshId)
    {
        if (mapper.meshIdToGameObjectMap.ContainsKey(meshId))
        {
            AddNavMesh(meshId, mapper.meshIdToGameObjectMap[meshId].GetComponent<MeshFilter>());
        }
    }
    #endregion

    #region GRAPH_INFO
    private bool ReadyForUpdate(NavMeshGraph graph)
    {
        float last;
        if (lastUpdate.TryGetValue(graph, out last))
        {
            return (Time.time - last) > minimumUpdateInterval;
        }
        return true;
    }

    private Vector3 GetCorner(NavMeshGraph graph)
    {
        center = graph.sourceMesh.bounds.center;
        return (new Vector3(
            Mathf.FloorToInt(center.x),
            Mathf.FloorToInt(center.y),
            Mathf.FloorToInt(center.z)
            ));
    }
    #endregion

    #region UPDATE_QUEUE
    private void QueueForUpdate(NavMeshGraph graph)
    {
        if ((updateQueue.Count >= updateQueueMaxItems || !ReadyForUpdate(graph)) && !toRemove.Contains(graph)) return;

        updateQueue.Enqueue(graph);
        if (!updateInProgress)
        {
            StartCoroutine(UpdateGraphRoutine(updateQueue.Dequeue()));
        }
    }

    private IEnumerator UpdateGraphRoutine(NavMeshGraph graph)
    {
        updateInProgress = true;

        pathing.AddWorkItem(() =>
        {
            UnstitchFromNeighbors(graph);
        });

        pathing.FlushWorkItems();
        while (pathing.IsAnyWorkItemInProgress) yield return null;

        if (toRemove.Contains(graph))
        {
            pathing.data.RemoveGraph(graph);


            toRemove.Remove(graph);
        }

        else
        {
            while (pathing.isScanning) yield return null;
            pathing.Scan(graph);
            while (pathing.isScanning) yield return null;



            pathing.AddWorkItem(() =>
            {
                StitchToNeighbors(graph);
            });
            pathing.FlushWorkItems();

            while (pathing.IsAnyWorkItemInProgress) yield return null;

            if (!lastUpdate.ContainsKey(graph)) lastUpdate.Add(graph, Time.time);
            else lastUpdate[graph] = Time.time;
        }

        updateInProgress = false;
        if (updateQueue.Count > 0)
        {
            StartCoroutine(UpdateGraphRoutine(updateQueue.Dequeue()));
        }
    }

    #endregion

    #region DEBUG
#if UNITY_EDITOR
//DEBUG
List<GraphNode> debugNodes = new List<GraphNode>();

private void OnDrawGizmos()
{
    foreach (GraphNode node in debugNodes)
    {
        Handles.color = Color.red;
        Handles.DrawSphere(0, (Vector3)node.position, Quaternion.identity, .05f);
        Handles.Label((Vector3)node.position, node.NodeIndex.ToString());
    }
    debugNodes.Clear();
    DrawStitches();

}
#endif
private void DrawStitches()
{
    if (idToNav == null) return;
    foreach (NavMeshGraph graph in idToNav.Values)
    {
        graph.GetNodes(node => DrawOffGraphConnections(node));
    }
}


void DrawOffGraphConnections(GraphNode node)
{
    node.GetConnections(otherNode => DrawOffGraphConnections(node, otherNode));
}

void DrawOffGraphConnections(GraphNode min, GraphNode other)
{
    if (min.GraphIndex != other.GraphIndex)
    {
        if (min.ContainsConnection(other))
            Debug.DrawLine((Vector3)min.position, (Vector3)other.position, Color.cyan); //two way
        else
            Debug.DrawLine((Vector3)min.position, (Vector3)other.position, Color.magenta); //one way
#if UNITY_EDITOR
        if (other.NodeIndex > 200000)
        {
            debugNodes.Add(other);
            Debug.LogError("--------- FOUND SUSPICIOUS NODE " + " from " + min.GraphIndex + " " + min.NodeIndex + " to " + other.GraphIndex + " " + other.NodeIndex + "   " + Time.time);
            other.GetConnections(node => PrintConnections(node));
        }
#endif
    }
}

void PrintConnections(GraphNode node)
{
    node.GetConnections(other => Debug.Log("----- node connected to " + other.NodeIndex));
}
#endregion
}