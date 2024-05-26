#include <stdio.h>
#include <math.h>

// Function to calculate the Euclidean distance between two colors in RGB space.
float color_distance(int r1, int g1, int b1, int r2, int g2, int b2)
{
    return sqrt(pow(r2 - r1, 2) + pow(g2 - g1, 2) + pow(b2 - b1, 2));
}

int main()
{
    // Suppose you have red, green and blue values.
    int red;
    int green;
    int blue;

    // Define standard colors and threshold for green
    int green_standard_red = 0;
    int green_standard_green = 255;
    int green_standard_blue = 0;
    float green_threshold = 200.0;

    // Define standard colors and threshold for red
    int red_standard_red = 255;
    int red_standard_green = 0;
    int red_standard_blue = 0;
    float red_threshold = 200.0;

    int another;
    do
    {
        printf("red: ");
        scanf("%d: ", &red);
        printf("green: ");
        scanf("%d: ", &green);
        printf("blue : ");
        scanf("%d: ", &blue);

        float green_distance = color_distance(red, green, blue, green_standard_red, green_standard_green, green_standard_blue);
        float red_distance = color_distance(red, green, blue, red_standard_red, red_standard_green, red_standard_blue);

        // Compare distances with thresholds and determine the closest color.
        if (green_distance < green_threshold && green_distance < red_distance)
        {
            printf("The color is more similar to green.\n");
        }
        else if (red_distance < red_threshold && red_distance < green_distance)
        {
            printf("The color is more similar to red.\n");
        }
        else
        {
            printf("Color is not in the range of green or red.\n");
        }
        printf("-----------------------\n");
        printf("green distance : %f \n", green_distance);
        printf("red distance : %f \n", red_distance);
        printf("-----------------------\n");
        printf("another: (y: 1, n: 0)");
        scanf("%d", &another);
    } while (another);

    return 0;
}
