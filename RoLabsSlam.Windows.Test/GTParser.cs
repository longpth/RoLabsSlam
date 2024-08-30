using System.Globalization;
using OpenCvSharp;

namespace RoLabsSlam.Windows.Test
{
    public class GTParser
    {
        public static List<Mat> Parse(string filePath)
        {
            // Check if the file path is valid and the file exists
            if (string.IsNullOrEmpty(filePath))
            {
                throw new ArgumentException("The file path is null or empty.");
            }

            if (!File.Exists(filePath))
            {
                throw new FileNotFoundException($"The file at path '{filePath}' was not found.");
            }

            var matrices = new List<Mat>();
            var lines = File.ReadAllLines(filePath);

            foreach (var line in lines)
            {
                // Split the line into individual elements
                var elements = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);

                if (elements.Length != 12)
                {
                    throw new FormatException("Each line must contain exactly 12 floating point numbers.");
                }

                // Create a new 3x4 Mat for the transformation matrix
                Mat matrix = new Mat(3, 4, MatType.CV_32F);

                // Fill the matrix with the parsed values
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        matrix.Set(i, j, float.Parse(elements[i * 4 + j], CultureInfo.InvariantCulture));
                    }
                }

                matrices.Add(matrix);
            }

            return matrices;
        }
    }

}
