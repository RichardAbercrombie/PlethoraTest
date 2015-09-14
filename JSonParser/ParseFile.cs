using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Runtime.Serialization.Json;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml;
using System.Xml.Linq;


namespace JSonParser
{
    /// <summary>
    /// JSON Serialization and Deserialization Assistant Class
    /// </summary>
    //public class JSonHelper
    //{  
    //    public string ConvertObjectToJSon<T>(T obj)
    //    {
    //        DataContractJsonSerializer ser = new DataContractJsonSerializer(typeof(T));
    //        MemoryStream ms = new MemoryStream();
    //        ser.WriteObject(ms, obj);

    //        string jsonString = Encoding.UTF8.GetString(ms.ToArray());
    //        ms.Close();

    //        return jsonString;
    //    }  

    //    public T ConvertJSonToObject<T>(string jsonString)
    //    {
    //        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(T));
    //        MemoryStream ms = new MemoryStream(Encoding.UTF8.GetBytes(jsonString));
    //        T obj = (T)serializer.ReadObject(ms);

    //        return obj;
    //    }
    //}


    /// <summary>
    /// Parse a .json file into memory
    /// </summary>
    public class ParseFile
    {
        /// <summary>
        /// The ObjectID of each edge
        /// </summary>
        private static List<int> Object_IDs;


        /// <summary>
        /// The VertexID's
        /// </summary>
        private static List<int> Vertex_IDs;


        /// <summary>
        /// The maximal laser cutting speed (inches/sec)
        /// </summary>
        private static double v_max = 0.5;


        /// <summary>
        /// Padding for laser kerf thickness (inches)
        /// </summary>
        private static double Padding = 0.1;


        /// <summary>
        /// The machine time cost per second (seconds)
        /// </summary>
        private static double MachineTimeCost = 0.07;


        /// <summary>
        /// Material cost per square inch
        /// </summary>
        private static double MaterialCost = 0.75;


        /// <summary>
        /// Container for the objects profile
        /// </summary>
        public ObjectProfile ObjProfile = new ObjectProfile();


        /// <summary>
        /// The vertices from the .json file with their Id
        /// </summary>
        public class Vertices
        {
            public int Id { get; set; }
            public double X { get; set; }
            public double Y { get; set; }
        }


        /// <summary>
        /// The Edges from the .json file for both
        /// CircularArcs and LineSegments
        /// </summary>
        public class Edge
        {
            public String Type { get; set; }
            public int Start { get; set; }
            public int End { get; set; }

            // ... for edge type CircularArc
            public int ClockWiseFrom { get; set; }
            public double CenterX { get; set; }
            public double CenterY { get; set; }
        }


        /// <summary>
        /// The Edge Id and edgeData
        /// </summary>
        public class ObjectId
        {
            public int Id { get; set; }
            public Edge edgedata { get; set; }
        }


        /// <summary>
        /// The main container for the objects profile in memory
        /// </summary>
        public class ObjectProfile
        {
            public List<ObjectId> Edges = new List<ObjectId>();
            public List<Vertices> Vertices = new List<Vertices>();
        }


        /// <summary>
        /// Take the Dot Product of two vectors
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        private static double DotProduct(double[] v1, double[] v2)
        {
            return (double)((v1[0] * v2[0]) + (v1[1] * v2[1]) + (v1[2] * v2[2]));
        }


        /// <summary>
        /// Normalize two vectors
        /// </summary>
        /// <param name="Vector"></param>
        /// <returns></returns>
        private static double[] Normalize(double[] Vector)
        {
            // ... get the length of this vector
            double length = (Vector[0] * Vector[0]) + (Vector[1] * Vector[1]) + (Vector[2] * Vector[2]);

            // ... prevent a divide by zero
            if (length == 0) return Vector;

            // ... only call division and sqrt function once
            double fac = 1 / Math.Sqrt(length);

            // ... normilize the vector
            Vector[0] = Vector[0] * fac;
            Vector[1] = Vector[1] * fac;
            Vector[2] = Vector[2] * fac;

            return Vector;
        }


        /// <summary>
        /// Calculate the vector length
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        private static double getVectorLength(double[] vector)
        {
            return Math.Sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
        }


        /// <summary>
        /// Calculate the distance between two points
        /// </summary>
        /// <param name="pt1"></param>
        /// <param name="pt2"></param>
        /// <returns></returns>
        private static double getDistance(double[] pt1, double[] pt2)
        {
            double X = pt1[0] - pt2[0];
            double Y = pt1[1] - pt2[1];
            double Z = pt1[2] - pt2[2];

            double Sq = (X * X) + (Y * Y) + (Z * Z);
            return Math.Sqrt(Sq);
        }


        /// <summary>
        /// Rotate point p by angle theta (radians) around point p2 (about Z axis)
        /// Positive angles are Clockwise looking down the axis
        /// </summary>
        /// <param name="p"></param>
        /// <param name="theta"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        private static double[] rotateAboutZ_Axis(double[] p, double theta, double[] p2)
        {
            double[] q = new double[3];

            double cosTheta = Math.Cos(theta);
            double sinTheta = Math.Sin(theta);

            // ... clockwise
            q[0] = p2[0] + (((p[0] - p2[0]) * cosTheta) + ((p[1] - p2[1]) * sinTheta));
            q[1] = p2[1] + ((-(p[0] - p2[0]) * sinTheta) + ((p[1] - p2[1]) * cosTheta));
            q[2] = p2[2] + p[2];

            return (q);
        }


        /// <summary>
        /// Returns the first number in string, 
        /// which is the ObjectID for this element
        /// </summary>
        /// <param name="str">input string</param>
        /// <returns>the ObjectID of the element</returns>
        private static int getObjectID(string str)
        {
            // ... split the string into a list of numbers
            string[] numbers = Regex.Split(str, @"\D+");

            // ... we only want the first number, which is the ObjectID
            foreach (string value in numbers)
            {
                // ... ignore Null or Empty strings
                if (!string.IsNullOrEmpty(value))
                {
                    // ... convert the string to an int
                    int j = int.Parse(value);

                    // ... return the value
                    return j;
                }
            }

            // ... return -1 on error
            return -1;
        }


        /// <summary>
        /// Returns the coordinates in the ObjectProfile pointed to by ptr
        /// </summary>
        /// <param name="profile"></param>
        /// <param name="ptr"></param>
        /// <returns></returns>
        private static List<double> getCoordinatesAt(ObjectProfile profile, int ptr)
        {
            // ... container to hold both x and y coordinates
            List<double> pos = new List<double>();

            // ... find the vertex we are after
            for (int i = 0; i < profile.Vertices.Count; i++)
            {
                // ... does the Id match ptr
                if (profile.Vertices[i].Id == ptr)
                {
                    // ... save the coordinates, x then y
                    pos.Add(profile.Vertices[i].X);
                    pos.Add(profile.Vertices[i].Y);

                    // ... we have the values, break the loop
                    break;
                }
            }

            // ... return the result
            return pos;
        }


        /// <summary>
        /// Calculate the cutting cost for all CircularArc's in this profile
        /// </summary>
        /// <param name="ObjProfile"></param>
        /// <returns></returns>
        public double getArcCuttingCost(ObjectProfile ObjProfile)
        {
            // ... the total perimeter length of the profile
            double ArcCuttingCost = 0.0;

            for (int i = 0; i < ObjProfile.Edges.Count; i++)
            {
                // ... get this Edge Type
                string str = ObjProfile.Edges[i].edgedata.Type;

                // ... we are only interested in Arcs at this point
                if (str.CompareTo("CircularArc") == 0)
                {
                    // ... get the X and Y vertices using the pointer for the start position
                    List<double> positionStart = getCoordinatesAt(ObjProfile, ObjProfile.Edges[i].edgedata.Start);

                    // ... get the X and Y vertices using the pointer for the end position
                    List<double> positionEnd = getCoordinatesAt(ObjProfile, ObjProfile.Edges[i].edgedata.End);

                    // ... the Arc may not always be 180 degrees, so calculate the arc length
                    double[] Vec1 = new double[3];
                    double[] Vec2 = new double[3];

                    // ... set Vector1
                    Vec1[0] = positionEnd[0] - ObjProfile.Edges[i].edgedata.CenterX;
                    Vec1[1] = positionEnd[1] - ObjProfile.Edges[i].edgedata.CenterY;

                    // ... set Vector2
                    Vec2[0] = positionStart[0] - ObjProfile.Edges[i].edgedata.CenterX;
                    Vec2[1] = positionStart[1] - ObjProfile.Edges[i].edgedata.CenterY;
                    Vec1[2] = Vec2[2] = 0.0;

                    // ... get the radius from this vector (before its normalize)
                    double Radius = getVectorLength(Vec1);

                    // ... normilize the vectors
                    Vec1 = Normalize(Vec1);  Vec2 = Normalize(Vec2);

                    // ... calculate the angle between the vectors (in radians)
                    double angle = Math.Acos(DotProduct(Vec1, Vec2) / (getVectorLength(Vec1) * getVectorLength(Vec2)));

                    // ... calculate the arc cutting cost = $((ArcLength / (v_max * exp(-1/R))) * Machine Time Cost)
                    ArcCuttingCost += ((angle * Radius) / (v_max * Math.Exp(-1 / Radius))) * MachineTimeCost;
                }
            }
            return ArcCuttingCost;
        }


        /// <summary>
        /// Calculates the Machine cost for cutting the parts straight (LineSegment) edges
        /// </summary>
        /// <param name="ObjProfile"></param>
        /// <returns></returns>
        public double getStraightEdgeCuttingCost(ObjectProfile ObjProfile)
        {
            // ... the total perimeter length of the profile
            double lengthCuttingCost = 0.0;

            // ... loop through all edges and calculate cutting cost for the LineSegment edge type
            for (int i = 0; i < ObjProfile.Edges.Count; i++)
            {
                // ... get this Edge Type
                string str = ObjProfile.Edges[i].edgedata.Type;

                if (str.CompareTo("LineSegment") == 0)
                {
                    // ... get the X and Y vertices using the pointer for the start position
                    List<double> positionStart = getCoordinatesAt(ObjProfile, ObjProfile.Edges[i].edgedata.Start);

                    // ... get the X and Y vertices using the pointer for the end position
                    List<double> positionEnd = getCoordinatesAt(ObjProfile, ObjProfile.Edges[i].edgedata.End);

                    // ... get the length of this edge (simple Pythagorean Theorem)
                    double A = positionEnd[0] - positionStart[0];
                    double B = positionEnd[1] - positionStart[1];
                    double length = Math.Sqrt((A * A) + (B * B));

                    // ... claculate the cost for this edge
                    lengthCuttingCost += (length / v_max) * MachineTimeCost;
                }
            }
            return lengthCuttingCost;
        }


        /// <summary>
        /// Calculates the midpoint of all CircularArcs relative to the Arcs start position and
        /// clockwise direction. These points are used to help calculate the parts material size.
        /// </summary>
        /// <param name="points"></param>
        /// <param name="ObjProfile"></param>
        private static void getMidPointOfAllArcs(List<PointF> points, ObjectProfile ObjProfile)
        {
            // ... loop through each edge and scan for CircularArc's
            for (int i = 0; i < ObjProfile.Edges.Count; i++)
            {
                // ... get this Edge Type
                string str = ObjProfile.Edges[i].edgedata.Type;

                // ... we are only interested in Arcs at this point
                if (str.CompareTo("CircularArc") == 0)
                {
                    // ... get the X and Y vertices using the pointer for the start position
                    List<double> positionStart = getCoordinatesAt(ObjProfile, ObjProfile.Edges[i].edgedata.Start);

                    // ... get the X and Y vertices using the pointer for the end position
                    List<double> positionEnd = getCoordinatesAt(ObjProfile, ObjProfile.Edges[i].edgedata.End);

                    // ... the Arc may not always be 180 degrees, so calculate the arc length
                    double[] Vec1 = new double[3];
                    double[] Vec2 = new double[3];

                    // ... set Vector1
                    Vec1[0] = positionEnd[0] - ObjProfile.Edges[i].edgedata.CenterX;
                    Vec1[1] = positionEnd[1] - ObjProfile.Edges[i].edgedata.CenterY;

                    // ... set Vector2
                    Vec2[0] = positionStart[0] - ObjProfile.Edges[i].edgedata.CenterX;
                    Vec2[1] = positionStart[1] - ObjProfile.Edges[i].edgedata.CenterY;
                    Vec1[2] = Vec2[2] = 0.0;

                    // ... normilize the vectors
                    Vec1 = Normalize(Vec1);  Vec2 = Normalize(Vec2);

                    // ... calculate the angle between the vectors (in radians)
                    double angle = Math.Acos(DotProduct(Vec1, Vec2) / (getVectorLength(Vec1) * getVectorLength(Vec2)));

                    // ... get the point we want to rotate
                    List<double> clockWiseFrom = getCoordinatesAt(ObjProfile, ObjProfile.Edges[i].edgedata.ClockWiseFrom);

                    // ... convert the clockWiseFrom
                    double[] p = new double[3];
                    p[0] = clockWiseFrom[0];
                    p[1] = clockWiseFrom[1];
                    p[2] = 0.0;

                    // ... center point to rotate about
                    double[] p2 = new double[3];
                    p2[0] = ObjProfile.Edges[i].edgedata.CenterX;
                    p2[1] = ObjProfile.Edges[i].edgedata.CenterY;
                    p2[2] = 0.0;

                    // ... rotate point p about the Z-Axis relative to the midpoint of the arc
                    double[] rotationPoint = new double[3];
                    rotationPoint = rotateAboutZ_Axis(p, (angle * 0.5), p2);

                    // ... convert to the PointF type
                    PointF pt = new PointF((float)rotationPoint[0], (float)rotationPoint[1]);

                    // ... save it to the list
                    points.Add(pt);
                }
            }
        }


        /// <summary>
        /// Calculates the (world) bounding box of the part, used to determine material size.
        /// 
        /// Ideally one would calculate a minimum bounding box for this using a Covariance Matrix,
        /// then solve for the eigenvalues and eigenvectors of a 3x3 symmetric matrix to determine
        /// the coordinate system.
        /// 
        /// </summary>
        /// <param name="pts"></param>
        /// <returns></returns>
        private static double[] calculateBoundingBox(List<PointF> pts)
        {
            // ... the lower left corner
            double[] low = new double[2];

            // ... initialize low to MaxValue
            low[0] = low[1] = double.MaxValue;

            // ... the upper right corner
            double[] high = new double[2];

            // ... initialize high to MinValue
            high[0] = high[1] = double.MinValue;

            // ... calculate the world bounding box around these points
            for (int i = 0; i < pts.Count; i++)
            {
                // ... check the low values
                if (pts[i].X < low[0])
                {
                    low[0] = pts[i].X;
                }
                if (pts[i].Y < low[1])
                {
                    low[1] = pts[i].Y;
                }

                // ... check the high values
                if (pts[i].X > high[0])
                {
                    high[0] = pts[i].X;
                }
                if (pts[i].Y > high[1])
                {
                    high[1] = pts[i].Y;
                }
            }

            // ... calculate the final size of the object
            double[] finalSize = new double[2];
            finalSize[0] = high[0] - low[0];
            finalSize[1] = high[1] - low[1];

            // ... return the value
            return finalSize;
        }


        /// <summary>
        /// Calculates the material size for costing
        /// </summary>
        /// <param name="ObjProfile"></param>
        /// <returns></returns>
        private static double[] getMaterialSize(ObjectProfile ObjProfile)
        {
            // ... container to hole the object x and y dimensions
            double[] size = new double[2];

            // ... container to hold all points for this profile
            List<PointF> points = new List<PointF>();

            // ... first store all vertices in the points collection
            for (int i = 0; i < ObjProfile.Vertices.Count; i++)
            {
                // ... get the point at this location
                PointF pt = new PointF((float)ObjProfile.Vertices[i].X, (float)ObjProfile.Vertices[i].Y);

                // ... save it to the points list
                points.Add(pt);
            }

            // ... calculate the midpoint of all Arcs
            getMidPointOfAllArcs(points, ObjProfile);

            // ... we now have enough points to calculate the bounding box of this part
            size = calculateBoundingBox(points);

            // ... return the values
            return size;
        }


        /// <summary>
        /// Calculates the material cost for this part
        /// </summary>
        /// <param name="ObjProfile"></param>
        /// <returns></returns>
        public double getMaterialCost(ObjectProfile ObjProfile)
        {
            // ... the size of the profile to be cut (x and y dimensions)
            double[] size = new double[2];

            // ... determine material size
            size = getMaterialSize(ObjProfile);

            // ... add padding to the size for the kerf offset
            size[0] += Padding;
            size[1] += Padding;

            // ... return the calculate the cost
            return ((size[0] * size[1]) * MaterialCost);
        }


        /// <summary>
        /// Parse the json file
        /// </summary>
        /// <param name="fileName"></param>
        public ParseFile(string fileName)
        {

            // ... read the file
            using (StreamReader r = new StreamReader(fileName))
            {
                // ... read the .json file
                string json = r.ReadToEnd();

                // ... create a new xmlDocument
                XmlDocument doc = new XmlDocument();

                // ... convert the data to xml format
                using (var reader = JsonReaderWriterFactory.CreateJsonReader(Encoding.UTF8.GetBytes(json), XmlDictionaryReaderQuotas.Max))
                {
                    XElement xml = XElement.Load(reader);
                    doc.LoadXml(xml.ToString());
                }

                // ... container to hold the edge ID's
                Object_IDs = new List<int>();

                // ... Get a list of Edges and extract the ObjectID of each
                XmlNodeList nodes0 = doc.DocumentElement.SelectNodes("//root//Edges");
                XmlNodeList l = nodes0[0].SelectNodes("//item");

                // ... Get the ObjectId for each element and store in a list
                for (int i = 0; i < l.Count; i++)
                {
                    XmlNode node = l[i];
                    string str = node.ParentNode.ParentNode.OuterXml.ToString();

                    // ... extract the ObjectID from the string
                    int ID = getObjectID(str);

                    // ... ensure there was no error
                    if (ID != -1)
                    {
                        // ... ensure we do not add the same edge twice
                        if (!Object_IDs.Contains(ID))
                        {
                            Object_IDs.Add(ID);
                        }
                    }
                }

                // ... loop through all Edges and save CircularArc's to the ObjectProfile
                XmlNodeList nodes1 = doc.DocumentElement.SelectNodes("//root//Edges");
                XmlNodeList lType = nodes1[0].SelectNodes("//Type");
                for (int i = 0; i < lType.Count; i++)
                {
                    XmlNode node = lType[i];

                    // ... create a new edge
                    Edge edge = new Edge();
                    edge.Type = node.InnerText;

                    if (edge.Type.CompareTo("CircularArc") == 0)
                    {
                        // ... get the CircularArc data from the file
                        XmlNodeList CenterX = nodes1[0].SelectNodes("//Center//X");
                        XmlNode CenNodeX = CenterX[0];

                        XmlNodeList CenterY = nodes1[0].SelectNodes("//Center//Y");
                        XmlNode CenNodeY = CenterY[0];

                        XmlNodeList cwFrom = nodes1[0].SelectNodes("//ClockwiseFrom");
                        XmlNode cwNode = cwFrom[0];

                        // ... save the values
                        edge.CenterX = Convert.ToDouble(CenNodeX.InnerText);
                        edge.CenterY = Convert.ToDouble(CenNodeY.InnerText);
                        edge.ClockWiseFrom = Convert.ToInt32(cwNode.InnerText);
                    }

                    // ... save the ObjectID and Edge
                    ObjectId ObjId = new ObjectId();
                    ObjId.Id = Object_IDs[i];
                    ObjId.edgedata = edge;

                    // ... add the Edge data to objects Profile class
                    ObjProfile.Edges.Add(ObjId);
                }

                // ... keep track of our position
                int currPosition = 0;

                // ... get the pointers to the vertex positions
                XmlNodeList nodes2 = doc.DocumentElement.SelectNodes("//root//Edges//Vertices");
                XmlNodeList lItem = nodes2[0].SelectNodes("//item");

                // ... store the edge pointers in the ObjectProfile
                for (int i = 0; i < lItem.Count; i++)
                {
                    XmlNode node = lItem[i];

                    // ... determine if we have the X or Y value
                    if (i % 2 != 0)
                    {
                        ObjProfile.Edges[currPosition].edgedata.End = int.Parse(node.InnerText);

                        // ... only increment after we have saved the Y value
                        currPosition++;
                    }
                    else
                    {
                        ObjProfile.Edges[currPosition].edgedata.Start = int.Parse(node.InnerText);
                    }
                }


                // ... container to hold the vertex ID's
                Vertex_IDs = new List<int>();

                // ... Get a list of Edges and extract the ObjectID of each
                foreach (XmlElement xmlElemenet in doc.DocumentElement.SelectNodes("//root//Vertices//item"))
                {
                    string str = xmlElemenet.OuterXml.ToString();

                    // ... extract the ObjectID from the string
                    int ID = getObjectID(str);

                    // ... ensure there was no error
                    if (ID != -1)
                    {
                        // ... ensure we do not add the same edge twice
                        if (!Vertex_IDs.Contains(ID))
                        {
                            Vertex_IDs.Add(ID);
                        }
                    }
                }

                // ... store the Vertices Position X and Y values in the ObjectProfile
                XmlNodeList nodes3 = doc.DocumentElement.SelectNodes("//root//Vertices//Position");
                for (int i = 0; i < nodes3.Count; i++)
                {
                    // ... get the X value
                    XmlNodeList lPosX = nodes3[i].SelectNodes("X");
                    XmlNode nodeX = lPosX[0];

                    // ... set the X value
                    Vertices verts = new Vertices();
                    verts.X = Convert.ToDouble(nodeX.InnerText);

                    // ... get the Y value
                    XmlNodeList lPosY = nodes3[i].SelectNodes("Y");
                    XmlNode nodeY = lPosY[0];

                    // ... convert the text to double
                    verts.Y = Convert.ToDouble(nodeY.InnerText);

                    // ... set the ID of this vertex
                    verts.Id = Vertex_IDs[i];

                    // ... save the vertex to the ObjectProfile
                    ObjProfile.Vertices.Add(verts);
                }

            }
        }
    }
}
