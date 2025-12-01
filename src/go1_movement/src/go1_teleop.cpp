#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Modo sin buffer y sin echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_key");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    geometry_msgs::Twist twist;

    puts("Control del robot:\n");
    puts("↑ : Avanzar");
    puts("↓ : Retroceder");
    puts("← : Girar izquierda");
    puts("→ : Girar derecha");
    puts("Espacio : Detener");

    ros::Rate rate(20); // 20 Hz

    while (ros::ok())
    {
        if (kbhit())
        {
            int c = getchar();
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            switch(c)
            {
                case 65: // flecha arriba
                    twist.linear.x = 0.5;
                    break;
                case 66: // flecha abajo
                    twist.linear.x = -0.5;
                    break;
                case 68: // flecha izquierda
                    twist.angular.z = 1.0;
                    break;
                case 67: // flecha derecha
                    twist.angular.z = -1.0;
                    break;
                case 32: // espacio
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    break;
                default:
                    break;
            }

            pub.publish(twist);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
