using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DisplayMeshGenerator : MonoBehaviour {

    [Range(0.1f,10f)]
    public float radius;
    [Range(30f, 180f)]
    public float xCurve = 72.4f, yCurve = 44.7f;
    [Range(2, 100)]
    public int xVerts = 32, yVerts = 18;
    private float xDelta, yDelta;

    public int webCamID;

    private void Start() {
        UpdateMesh();
        UpdateCamTexture();        
    }

    private void OnValidate() {        
        UpdateMesh();
    }

    void UpdateMesh() {
        xDelta = xCurve / xVerts;
        yDelta = yCurve / yVerts;

        Mesh mesh = new Mesh();
        MeshRepresentor meshRep = GetVertices();

        int[] triangles = GetTriangles(meshRep);

        mesh.vertices = meshRep.vertexList;
        mesh.triangles = triangles;
        mesh.uv = meshRep.uvs;

        GetComponent<MeshFilter>().mesh = mesh;
    }

    void UpdateCamTexture() {
        WebCamDevice[] devices = WebCamTexture.devices;
        WebCamTexture feed = new WebCamTexture(devices[webCamID].name, 1920, 1080, 60);
        Material surface = GetComponent<Renderer>().material;
        surface.mainTexture = feed;
        feed.Play();
    }

    class Vertex {
        public int index;
        public Vector3 position;

        public Vertex(int i, Vector3 p) {
            index = i;
            position = p;
        }
    }

    class MeshRepresentor {
        public int x, y;
        public Vector3[] vertexList;
        public Vertex[,] vertexMap;
        public Vector2[] uvs;

        public MeshRepresentor(int x, int y, Vector3[] l, Vertex[,] m, Vector2[] u) {
            this.x = x;
            this.y = y;
            vertexList = l;
            vertexMap = m;
            uvs = u;
        }
    }

    int[] GetTriangles(MeshRepresentor meshRep) {
        // number of squares, times two, with three corners!
        int triCount = (meshRep.x - 1) * (meshRep.y - 1) * 2 * 3;
        int[] triangles = new int[triCount];

        int t = 0;
        for (int i = 0; i < meshRep.x - 1; i++) {
            for (int j = 0; j < meshRep.y - 1; j++) {
                //(i,j) is the square to the top right of vertex (x=i,y=j)
                triangles[t + 2] = meshRep.vertexMap[i, j].index;
                triangles[t + 1] = meshRep.vertexMap[i, j + 1].index;
                triangles[t] = meshRep.vertexMap[i + 1, j + 1].index;

                triangles[t + 5] = meshRep.vertexMap[i, j].index;
                triangles[t + 4] = meshRep.vertexMap[i + 1, j + 1].index;
                triangles[t + 3] = meshRep.vertexMap[i + 1, j].index;

                t += 6;
            }
        }

        return triangles;
    }

    MeshRepresentor GetVertices() {
        int vertexCount = xVerts * yVerts;
        
        Vector3[] vertices = new Vector3[vertexCount];
        Vertex[,] vertexMap = new Vertex[xVerts, yVerts];
        Vector2[] uvs = new Vector2[vertexCount];

        Vector3 normal = Vector3.forward * radius;

        int count = 0;
        for (int i = 0; i < xVerts; i++) {
            for (int j = 0; j < yVerts; j++) {
                float xI = i - ((xVerts - 1) / 2f);
                float yI = j - ((yVerts - 1) / 2f);

                float xA = xI * xDelta;
                float yA = yI * yDelta;

                // rotating around x axis results in translation in y! Vice versa.
                Quaternion rotation = Quaternion.Euler(new Vector3(yA, xA, 0));
                Vector3 position = rotation * normal;

                /*
                GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                marker.name = "(" + i.ToString() + "," + j.ToString() + ")";
                marker.transform.position = position;
                marker.transform.localScale = Vector3.one * 0.05f;
                */

                Vertex vertex = new Vertex(count, position);
                vertices[count] = position;

                float u = i / (float)(xVerts - 1);
                float v = 1 - (j / (float)(yVerts - 1));
                uvs[count] = new Vector2(u, v);
                
                vertexMap[i, j] = vertex;

                count++;
            }
        }

        return new MeshRepresentor(xVerts, yVerts, vertices, vertexMap, uvs);
    }


}
