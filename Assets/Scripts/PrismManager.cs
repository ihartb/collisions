using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PrismManager : MonoBehaviour
{
    public int prismCount = 10;
    public float prismRegionRadiusXZ = 5;
    public float prismRegionRadiusY = 5;
    public float maxPrismScaleXZ = 5;
    public float maxPrismScaleY = 5;
    public GameObject regularPrismPrefab;
    public GameObject irregularPrismPrefab;

    private List<Prism> prisms = new List<Prism>();
    private List<GameObject> prismObjects = new List<GameObject>();
    private GameObject prismParent;
    private Dictionary<Prism,bool> prismColliding = new Dictionary<Prism, bool>();

    private const float UPDATE_RATE = 0.5f;

    #region Unity Functions

    void Start()
    {
        Random.InitState(0);    //10 for no collision

        prismParent = GameObject.Find("Prisms");
        for (int i = 0; i < prismCount; i++)
        {
            var randPointCount = Mathf.RoundToInt(3 + Random.value * 7);
            var randYRot = Random.value * 360;
            var randScale = new Vector3((Random.value - 0.5f) * 2 * maxPrismScaleXZ, (Random.value - 0.5f) * 2 * maxPrismScaleY, (Random.value - 0.5f) * 2 * maxPrismScaleXZ);
            var randPos = new Vector3((Random.value - 0.5f) * 2 * prismRegionRadiusXZ, (Random.value - 0.5f) * 2 * prismRegionRadiusY, (Random.value - 0.5f) * 2 * prismRegionRadiusXZ);

            GameObject prism = null;
            Prism prismScript = null;
            if (Random.value < 0.5f)
            {
                prism = Instantiate(regularPrismPrefab, randPos, Quaternion.Euler(0, randYRot, 0));
                prismScript = prism.GetComponent<RegularPrism>();
            }
            else
            {
                prism = Instantiate(irregularPrismPrefab, randPos, Quaternion.Euler(0, randYRot, 0));
                prismScript = prism.GetComponent<IrregularPrism>();
            }
            prism.name = "Prism " + i;
            prism.transform.localScale = randScale;
            prism.transform.parent = prismParent.transform;
            prismScript.pointCount = randPointCount;
            prismScript.prismObject = prism;

            prisms.Add(prismScript);
            prismObjects.Add(prism);
            prismColliding.Add(prismScript, false);
        }

        StartCoroutine(Run());
    }
    
    void Update()
    {
        #region Visualization

        DrawPrismRegion();
        DrawPrismWireFrames();

#if UNITY_EDITOR
        if (Application.isFocused)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
#endif

        #endregion
    }

    IEnumerator Run()
    {
        yield return null;

        while (true)
        {
            foreach (var prism in prisms)
            {
                prismColliding[prism] = false;
            }

            foreach (var collision in PotentialCollisions())
            {
                if (CheckCollision(collision))
                {
                    prismColliding[collision.a] = true;
                    prismColliding[collision.b] = true;

                    ResolveCollision(collision);
                }
            }

            yield return new WaitForSeconds(UPDATE_RATE);
        }
    }

    #endregion

    #region Incomplete Functions

    private IEnumerable<PrismCollision> PotentialCollisions()
    {
        var sortedXValues = new List<Tuple<float, Prism>>();

        //find the min and max points of the AABB for each prism 
        for (int i = 0; i < prisms.Count; i++)
        {
            Prism shape = prisms[i];
            shape.min = new Vector3(shape.points.Min(p => p.x), 0, shape.points.Min(p=> p.z));
            shape.max = new Vector3(shape.points.Max(p => p.x), 0, shape.points.Max(p => p.z));

            //add the min x and max x values to sort & sweep list
            sortedXValues.Add(new Tuple<float, Prism>(shape.min.x, shape));
            sortedXValues.Add(new Tuple<float, Prism>(shape.max.x, shape));
        }

        //sort the x values in list
        sortedXValues = sortedXValues.OrderBy(pairing => pairing.Item1).ToList();

        var active = new HashSet<Prism>();
        foreach (var pair in sortedXValues) //sweep to find potential collisions

        {
            if (active.Contains(pair.Item2)) //if we reach the max x value of a prism
            {
                active.Remove(pair.Item2);
                // check all prisms in active list and see if y values overlap with 
                foreach (var shape in active) 
                {
                    if (IntersectingZ(pair.Item2, shape)) //y values overlap so AABB intersect --> potential collision
                    {
                        var checkPrisms = new PrismCollision();
                        checkPrisms.a = pair.Item2;
                        checkPrisms.b = shape;

                        yield return checkPrisms;
                    }
                }
            }
            else //min x value, add to active list
            {
                active.Add(pair.Item2);
            }
        }
        yield break;
    }

    private bool IntersectingZ(Prism a, Prism b)
    {
        return (a.max.z > b.min.z && a.min.z < b.max.z) || (b.max.z > a.min.z && b.min.z < a.max.z);
    }

    private bool CheckCollision(PrismCollision collision)
    {
        var prismA = collision.a;
        var prismB = collision.b;
        collision.penetrationDepthVectorAB = Vector3.zero;

        var minkDiff = minkowskiDifference(prismA, prismB);

        var simplex = new List<Vector3>();
        simplex.Add(minkDiff.Aggregate((p1, p2) => p1.x < p2.x ? p1 : p2));
        simplex.Add(minkDiff.Where(p => p != simplex[0]).Aggregate((p1, p2) => p1.x > p2.x ? p1 : p2));
        var intersecting = GJKAlgo(minkDiff, simplex);

        if (intersecting)
        {
            collision.penetrationDepthVectorAB = EPAAlgo(minkDiff, simplex);
            //Debug.DrawLine(prismA.transform.position, prismB.transform.position - tan, Color.magenta, UPDATE_RATE);
        }

        return intersecting;
    }

    private Vector3 EPAAlgo(List<Vector3>minkDiff, List<Vector3> simplex)
    {
        if (PointToLine(simplex[0], simplex[1], simplex[2]) > 0) //consistency of CC or clockwise 
        {
            var temp = simplex[0];
            simplex[0] = simplex[1];
            simplex[1] = temp;
        }

        var distToSegments = new List<float>(); //list of distance from each segment to origin

        for (int i = 0; i < simplex.Count; i++) //populate distToSegments
        {
            var a = simplex[i];
            var b = simplex[(i + 1) % simplex.Count];
            //Debug.DrawLine(a, b, Color.magenta, UPDATE_RATE);
            distToSegments.Add(Mathf.Abs(PointToLine(Vector3.zero, a, b)));
        }

        var minIndex = MinIndex(distToSegments);
        var minDist = distToSegments[minIndex];
        for (int k = 0; k < 10; k++)
        {
            //find point along line perpendicular to two points thats closest to origin 
            var dir = simplex[(minIndex + 1) % simplex.Count] - simplex[minIndex];
            var tangent = Vector3.Cross(dir, Vector3.up);
            var orientation = -Mathf.Sign(Vector3.Dot(tangent, simplex[minIndex])); //if i make -simplex[minIndex] we dont get the minimum!
            var supportAxis = tangent * orientation;
            var supportPoint = minkDiff.Aggregate((p1, p2) =>
                Vector3.Dot(p1, supportAxis) > Vector3.Dot(p2, supportAxis) ? p1 : p2);

            if (simplex.Contains(supportPoint))
            {
                print("break");
                break;
            }
            else //ISSUE NEVER GOES HERE, ALWAYS BREAKS!
            {
                print("new min");
                var ind = (minIndex + 1) % simplex.Count;
                simplex.Insert(ind, supportPoint);
                distToSegments.Insert(ind, float.MaxValue);

                minIndex = MinIndex(distToSegments);
                for (int j = minIndex; j <= minIndex; j++)
                {
                    var a = simplex[j];
                    var b = simplex[(j + 1) % simplex.Count];
                    distToSegments[(j % simplex.Count)] = Mathf.Abs(PointToLine(Vector3.zero, a, b));
                }

                minIndex = MinIndex(distToSegments);
                minDist = distToSegments[minIndex];
            }
        }

        return PointToLineTangent(Vector3.zero, simplex[minIndex], simplex[(minIndex + 1) % simplex.Count]);
    }

    private int MinIndex(List<float> vals)
    {
        int pos = 0;
        for (int i = 0; i < vals.Count; i++)
        {
            if (vals[i] < vals[pos]) { pos = i; }
        }

        return pos;
    }
    private float PointToLine(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        var res = Mathf.Sign(Vector3.Dot(
                    Vector3.Cross(p3 - p2, Vector3.up), p1 - p2));
        return res;
    }
    private Vector3 PointToLineTangent(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        var vect = p1 - p2;
        var dir = p3 - p2;
        var tangent = Vector3.Cross(dir, Vector3.up).normalized;
        var res = Vector3.Dot(vect, tangent) / (vect.magnitude) * vect.magnitude;
        return res * tangent;
    }
    private bool GJKAlgo(List<Vector3> minkDiff, List<Vector3> simplex)
    {
        var intersecting = false;
        do
        {
            if (simplex.Count == 3) //check if convex hull (triangle) contains origin
            {
                var simplexOrientation = Mathf.Sign(Vector3.Dot(
                    Vector3.Cross(simplex[1] - simplex[0], Vector3.up), simplex[2] - simplex[0]));

                var containsOrigin = true;
                Vector3? removePoint = null;
                for (int i = 0; i < 3; i++)
                {
                    var a = simplex[i];
                    var b = simplex[(i + 1) % 3];
                    var temp = Mathf.Sign(Vector3.Dot(
                        Vector3.Cross(b - a, Vector3.up), Vector3.zero - a));

                    if (temp != simplexOrientation)
                    {
                        containsOrigin = false;
                        removePoint = simplex[(i + 2) % 3];
                        break;
                    }
                }
                if (containsOrigin)
                {
                    intersecting = true;
                    break;
                }
                else if (removePoint.HasValue) //does not contain origin, remove point that is useless
                {
                    simplex.Remove(removePoint.Value);
                }
            }

            var dir = simplex[1] - simplex[0];
            var tangent = Vector3.Cross(dir, Vector3.up);
            var orientation = Mathf.Sign(Vector3.Dot(tangent, -simplex[0]));
            var supportAxis = tangent * orientation;
            var supportPoint = minkDiff.Aggregate((p1, p2) =>
                Vector3.Dot(p1, supportAxis) > Vector3.Dot(p2, supportAxis) ? p1 : p2);

            if (!simplex.Contains(supportPoint))
            {
                simplex.Add(supportPoint);
            }

        } while (simplex.Count == 3);

        return intersecting;
    }

    private List<Vector3> minkowskiDifference(Prism a, Prism b)
    {
        var minkDiff = new List<Vector3>();
        foreach (var p1 in a.points)
        {
            foreach (var p2 in b.points)
            {
                var res = p1 - p2;
                minkDiff.Add(res);
                //Debug.DrawLine(Vector3.zero, Vector3.zero + Vector3.up * 3, Color.red, UPDATE_RATE);
                //Debug.DrawLine(res, res + Vector3.up * 3, Color.yellow, UPDATE_RATE);
            }
        }
        return minkDiff;
    }
    
    #endregion

    #region Private Functions
    
    private void ResolveCollision(PrismCollision collision)
    {
        var prismObjA = collision.a.prismObject;
        var prismObjB = collision.b.prismObject;

        var pushA = -collision.penetrationDepthVectorAB / 2;
        var pushB = collision.penetrationDepthVectorAB / 2;

        for (int i = 0; i < collision.a.pointCount; i++)
        {
            collision.a.points[i] += pushA;
        }
        for (int i = 0; i < collision.b.pointCount; i++)
        {
            collision.b.points[i] += pushB;
        }
        //prismObjA.transform.position += pushA;
        //prismObjB.transform.position += pushB;

        Debug.DrawLine(prismObjA.transform.position, prismObjA.transform.position + collision.penetrationDepthVectorAB, Color.cyan, UPDATE_RATE);
    }
    
    #endregion

    #region Visualization Functions

    private void DrawPrismRegion()
    {
        var points = new Vector3[] { new Vector3(1, 0, 1), new Vector3(1, 0, -1), new Vector3(-1, 0, -1), new Vector3(-1, 0, 1) }.Select(p => p * prismRegionRadiusXZ).ToArray();
        
        var yMin = -prismRegionRadiusY;
        var yMax = prismRegionRadiusY;

        var wireFrameColor = Color.yellow;

        foreach (var point in points)
        {
            Debug.DrawLine(point + Vector3.up * yMin, point + Vector3.up * yMax, wireFrameColor);
        }

        for (int i = 0; i < points.Length; i++)
        {
            Debug.DrawLine(points[i] + Vector3.up * yMin, points[(i + 1) % points.Length] + Vector3.up * yMin, wireFrameColor);
            Debug.DrawLine(points[i] + Vector3.up * yMax, points[(i + 1) % points.Length] + Vector3.up * yMax, wireFrameColor);
        }
    }

    private void DrawPrismWireFrames()
    {
        for (int prismIndex = 0; prismIndex < prisms.Count; prismIndex++)
        {
            var prism = prisms[prismIndex];
            var prismTransform = prismObjects[prismIndex].transform;

            var yMin = prism.midY - prism.height / 2 * prismTransform.localScale.y;
            var yMax = prism.midY + prism.height / 2 * prismTransform.localScale.y;

            var wireFrameColor = prismColliding[prisms[prismIndex]] ? Color.red : Color.green;

            foreach (var point in prism.points)
            {
                Debug.DrawLine(point + Vector3.up * yMin, point + Vector3.up * yMax, wireFrameColor);
            }

            for (int i = 0; i < prism.pointCount; i++)
            {
                Debug.DrawLine(prism.points[i] + Vector3.up * yMin, prism.points[(i + 1) % prism.pointCount] + Vector3.up * yMin, wireFrameColor);
                Debug.DrawLine(prism.points[i] + Vector3.up * yMax, prism.points[(i + 1) % prism.pointCount] + Vector3.up * yMax, wireFrameColor);
            }
        }
    }
    #endregion

    #region Utility Classes

    private class PrismCollision
    {
        public Prism a;
        public Prism b;
        public Vector3 penetrationDepthVectorAB;
    }

    private class Tuple<K,V>
    {
        public K Item1;
        public V Item2;

        public Tuple(K k, V v) {
            Item1 = k;
            Item2 = v;
        }
    }

    #endregion
}
