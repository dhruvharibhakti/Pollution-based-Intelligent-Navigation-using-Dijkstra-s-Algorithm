#include <FirebaseArduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
 #include <ESP8266HTTPClient.h>

#define FIREBASE_HOST "intelligent-navigation-system.firebaseio.com"
//#define FIREBASE_HOST "https://capstone-ec3da.firebaseio.com/"
//#define FIREBASE_AUTH "bRTukmk60taMG3mIyjpXA08MjlhDJ5ZnRT40KF9W"
#define WIFI_SSID "NETGEAR" // Change the name of your WIFI
#define WIFI_PASSWORD "sah_1407" // Change the password of your WIFI

#define MUX_A D4
#define MUX_B D3
#define MUX_C D2
#define ANALOG_INPUT A0
#define INFINITY 9999
#define MAX 6

String myString;

void setup() {
  Serial.begin(115200);
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);     
  pinMode(MUX_C, OUTPUT);
  WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println ("");
  Serial.println ("WiFi Connected!");
  //Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.begin(FIREBASE_HOST);
}

float dijkstra(int G[MAX][MAX],int n,int startnode,int endnode)
{
  startnode=startnode-1;
  endnode=endnode-1;
  int cost[MAX][MAX],distance[MAX],pred[MAX];
  int visited[MAX],count,mindistance,nextnode,i,j;
  
  //pred[] stores the predecessor of each node
  //count gives the number of nodes seen so far
  //create the cost matrix
  for(i=0;i<n;i++)
    for(j=0;j<n;j++)
      if(G[i][j]==0)
        cost[i][j]=INFINITY;
      else
        cost[i][j]=G[i][j];
  
  //initialize pred[],distance[] and visited[]
  for(i=0;i<n;i++)
  {
    distance[i]=cost[startnode][i];
    pred[i]=startnode;
    visited[i]=0;
  }
  
  distance[startnode]=0;
  visited[startnode]=1;
  count=1;
  while(count<n-1)
  {
    mindistance=INFINITY;

    //nextnode gives the node at minimum distance
    for(i=0;i<n;i++)
      if(distance[i]<mindistance&&!visited[i])
      {
        mindistance=distance[i];
        nextnode=i;
      }
      
      //check if a better path exists through nextnode      
      visited[nextnode]=1;
      for(i=0;i<n;i++)
        if(!visited[i])
          if(mindistance+cost[nextnode][i]<distance[i])
          {
            distance[i]=mindistance+cost[nextnode][i];
            pred[i]=nextnode;
          }
    count++;
  }

  //print the path and distance of each node
  Serial.print("Total Cost: ");
  Serial.println(distance[endnode]);
  Serial.print("Best Route: ");
  Serial.print(endnode+1);
  j=endnode;
  do
  {
        j=pred[j];
        Serial.print("<-");
        Serial.print(j+1);
  }while(j!=startnode);
  Serial.println();
  return(distance[endnode]);
}

void changeMux(int c, int b, int a) {
  digitalWrite(MUX_A, a);
  digitalWrite(MUX_B, b);
  digitalWrite(MUX_C, c);
}

void loop() {
  float value;
  boolean state[6][3] = {{LOW, LOW, LOW},{LOW, LOW, HIGH},{LOW, HIGH, LOW},{LOW, HIGH, HIGH},{HIGH, LOW, LOW},{HIGH, LOW, HIGH}};
  int i;
  float poll[6];
  
    for(i=0;i<6;i++){
      changeMux(state[i][0], state[i][1], state[i][2]);
      value = analogRead(ANALOG_INPUT);
      poll[i] = (0.6*value);
      Serial.print(i+1);
      Serial.print(". PPM: ");
      Serial.println(value);
      delay(500);
    }
    int G[6][6] = {{0,(0.4*14)+99999+poll[0]+poll[1],(0.4*9)+poll[0]+poll[2],(0.4*7)+poll[0]+poll[3],0,0},
    {(0.4*14)+99999+poll[1]+poll[0],0,(0.4*2)+poll[1]+poll[2],0,(0.4*9)+poll[1]+poll[4],0},
    {(0.4*9)+poll[2]+poll[0],(0.4*2)+poll[2]+poll[1],0,(0.4*10)+poll[2]+poll[3],0,(0.4*11)+poll[2]+poll[5]},
    {(0.4*7)+poll[3]+poll[0],0,(0.4*10)+poll[3]+poll[2],0,0,(0.4*15)+poll[3]+poll[5]},
    {0,(0.4*9)+poll[4]+poll[1],0,0,0,(0.4*6)+poll[4]+poll[5]},
    {0,0,(0.4*11)+poll[5]+poll[2],(0.4*15)+poll[5]+poll[3],(0.4*6)+poll[5]+poll[4],0}};
    int G1[6][6] = {{0,14,9,7,0,0},{14,0,2,0,9,0},{9,2,0,10,0,11},{7,0,10,0,0,15},{0,9,0,0,0,6},{0,0,11,15,6,0}};         //test
    
    Serial.println("Source= 1\nDestination= 5");
    Serial.println("Cost and Best Route WITH considering Pollution");
    float d = dijkstra(G,6,1,5);
    //char* d = dijkstra(G,6,1,5);
    //dijkstra(G,6,1,5);
    Serial.println("Cost and Best Route WITHOUT considering Pollution");
    dijkstra(G1,6,1,5);
    Serial.println("--------------------------------------------------------------");
    //Serial.println(d);
    myString = String(d);
    Firebase.setString("Distance",myString);
    delay(2000);
}
