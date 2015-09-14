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
    /// Plethora Technical Exercise Program
    /// </summary>
    public class Program
    {
        // ... the main entry point
        static void Main(string[] args)
        {
            // ... info for key press
            ConsoleKeyInfo cki;

            // ... clear the console
            Console.Clear();

            while (true)
            {
                // ... User input filename, including path
                Console.WriteLine("Input fileName to parse, including full path: ");
                Console.WriteLine();

                // ... read the path from the command line
                string fileName = Console.ReadLine();

                // ... check that the file path is valid
                if (File.Exists(fileName))
                {
                    // ... parse the file
                    ParseFile jsonReader = new ParseFile(fileName);

                    // ... get the cost for cutting the straight edges
                    double Cost = jsonReader.getStraightEdgeCuttingCost(jsonReader.ObjProfile);

                    // ... sum to this the cost for cutting the Circular Arcs
                    Cost += jsonReader.getArcCuttingCost(jsonReader.ObjProfile);

                    // ... sum to this the material cost (includes kerf offset)
                    Cost += jsonReader.getMaterialCost(jsonReader.ObjProfile);

                    // ... print the final cost
                    Console.WriteLine("Cost to produce part: ${0}", Cost);
                    Console.WriteLine();

                    // ... print exit method to screen
                    Console.Write("Press any key, or 'X' to quit ");
                    Console.WriteLine();

                    // ... Start a console read operation
                    cki = Console.ReadKey(true);

                    // ... Exit if the user pressed the 'X' key. 
                    if (cki.Key == ConsoleKey.X) break;

                }
                else
                {
                    Console.WriteLine("{0} is not a valid file.", fileName);
                }

            }

        }
    }
}
