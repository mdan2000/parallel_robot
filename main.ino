#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
//192.1168.4.1
#define D2 4
#define D4 2
#define SERVO_STEP 5

const char* ssid = "DEXTAR";
const char* password = "114_robots";

ESP8266WebServer server(80);
String IP;
Servo servo_L;
Servo servo_R;

double servo_L_angle = 90.0;
double servo_R_angle = 90.0;


void handleRoot();
void handleRootPOST();

void setup() {
  Serial.begin(115200);
  servo_L.attach(D2);
  servo_R.attach(D4);

  servo_L.write(servo_L_angle);
  servo_R.write(servo_R_angle);
  
  WiFi.softAP(ssid, password);
  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());
  IP = WiFi.softAPIP().toString();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/", HTTP_POST, handleRootPOST);
  
  server.begin();
  Serial.println("HTTP server started");
}

double EPS = 0.0000001;

double quad(double p) {
  return p*p;
}

double dist(double x1, double y1, double x2, double y2) {
  return sqrt(quad(x1 - x2) + quad(y1 - y2));
}

void intersection(double r, double a, double b, double c, int side, double dx, double dy) {
  if (side == 0) {
    Serial.print(" left ");
  } else {
    Serial.print(" right ");
  }
  Serial.print(" a:");
  Serial.print(a);
  Serial.print(" b:");
  Serial.print(b);
  Serial.print(" c:");
  Serial.print(c);
  Serial.print(" ");
  double x0 = -a*c/(a*a+b*b),  y0 = -b*c/(a*a+b*b);
  if (c*c > r*r*(a*a+b*b)+EPS) {
    Serial.println("no points");
  }
  else if (abs(c*c - r*r*(a*a+b*b)) < EPS) {
    Serial.print("1 point ");
    Serial.print(x0);
    Serial.print(" ");
    Serial.println(y0);
  }
  else {
    double d = r*r - c*c/(a*a+b*b);
    double mult = sqrt (d / (a*a+b*b));
    double ax,ay,bx,by;
    ax = x0 + b * mult;
    bx = x0 - b * mult;
    ay = y0 - a * mult;
    by = y0 + a * mult;
    Serial.print("2 points ");
    Serial.print(ax);
    Serial.print(" ");
    Serial.print(ay);
    Serial.print(" and ");
    Serial.print(bx);
    Serial.print(" ");
    Serial.println(by);
    double xnow, ynow;
    double angnow;
    ax += dx;
    ay += dy;
    bx += dx;
    by += dy;
    if (side == 0) {
      angnow = servo_L.read();
    } else {
      angnow = servo_R.read();
    }
    if (angnow < 90) {
      if (side == 0) {
        xnow = -4.5 + 7.5 * cos(angnow);
      } else {
        xnow = 4.5 + 7.5 * cos(angnow);
      }
      ynow = sin(angnow) * 7.5;
    } else {
      if (side == 0) {
        xnow = -4 - 7.5 * cos(180 - angnow);
      } else {
        xnow = 4 - 7.5 * cos(180 - angnow);
      }
      ynow = -10.5 + sin(180 - angnow) * 7.5;
    }
    if (dist(xnow, ynow, ax, ay) < dist(xnow, ynow, bx, by)) {
      Serial.print("need dot ");
      Serial.print(ax);
      Serial.print(" ");
      Serial.println(ay);
      if (side == 0) {
        double hipot = 7.5;
        Serial.print("Delta x ");
        Serial.println(dist(4, 0, ax, 0));
        double cos_angle = dist(-4, 0, ax, 0) / hipot;
        if (ax > -4) {
          servo_L.write(degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(degrees(acos(cos_angle)));
        } else {
          servo_L.write(180 - degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(180 - degrees(acos(cos_angle)));
        }
      } else {
        double hipot = 7.5;
        Serial.print("Delta x ");
        Serial.println(dist(4, 0, ax, 0));
        double cos_angle = dist(4, 0, ax, 0) / hipot;
        if (ax > 4) {
          servo_R.write(degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(degrees(acos(cos_angle)));
        } else {
          servo_R.write(180 - degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(180 - degrees(acos(cos_angle)));
        }
      }
    } else {
      Serial.print("need dot ");
      Serial.print(bx);
      Serial.print(" ");
      Serial.println(by);
      if (side == 0) {
        double hipot = 7.5;
        Serial.print("Delta x ");
        Serial.println(dist(4, 0, bx, 0));
        double cos_angle = dist(-4, 0, bx, 0) / hipot;
        if (bx > -4) {
          servo_L.write(degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(degrees(acos(cos_angle)));
        } else {
          servo_L.write(180 - degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(180 - degrees(acos(cos_angle)));
        }
      } else {
        double hipot = 7.5;
        Serial.print("Delta x ");
        Serial.println(dist(4, 0, bx, 0));
        double cos_angle = dist(4, 0, bx, 0) / hipot;
        if (bx > 4) {
          servo_R.write(degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(degrees(acos(cos_angle)));
        } else {
          servo_R.write(180 - degrees(acos(cos_angle)));
          Serial.print("angle is ");
          Serial.println(180 - degrees(acos(cos_angle)));
        }
      }
    }
  }
}

void rotate_from_zero(double dx, double dy) {
  Serial.println("test");
  double mid = 90;
  delay(2000);
  double small_r = 5;
  double big_r = 7.5;
  double x_l_servo = -4, y_l_servo = -10.5;
  double x_r_servo = 4, y_r_servo = -10.5;
  double x_l_point, y_l_point;
  double x_r_point, y_r_point;
  x_l_servo -= dx; y_l_servo -= dy;
  x_r_servo -= dx; y_r_servo -= dy;
  double a_l = -2 * x_l_servo;
  double b_l = -2 * y_l_servo;
  double c_l = quad(x_l_servo) + quad(y_l_servo) + quad(small_r) - quad(big_r);
  intersection(small_r, a_l, b_l, c_l, 0, dx, dy);
  double a_r = -2 * x_r_servo;
  double b_r = -2 * y_r_servo;
  double c_r = quad(x_r_servo) + quad(y_r_servo) + quad(small_r) - quad(big_r);
  intersection(small_r, a_r, b_r, c_r, 1, dx, dy);
}

void loop() {
  server.handleClient();
  delay(5000);
  Serial.println("Now to -0 -0");
  rotate_from_zero(0, 0);
  delay(5000);
  Serial.println("Now to -3 -3");
  rotate_from_zero(-3, -3);
}

void handleRoot() {
  
  server.send(200, "text/html", "<html>\
                                  <body>\
                                  <script type=\"text/javascript\">\
                                    function reply_click(clicked_id)\
                                    {\
                                        var xhr = new XMLHttpRequest();\
                                        var data = encodeURIComponent(clicked_id);\
                                        xhr.open(\"POST\", \"\", true);\
                                        xhr.setRequestHeader(\'Content-Type\', 'application/x-www-form-urlencoded');\
                                        xhr.send(clicked_id);\
                                    }\
                                  </script>\
                                  <button id=\"1\" onClick=\"reply_click(this.id)\">L -5grad</button>\
                                  <button id=\"2\" onClick=\"reply_click(this.id)\">L +5grad</button>\
                                  <button id=\"3\" onClick=\"reply_click(this.id)\">R -5grad</button>\
                                  <button id=\"4\" onClick=\"reply_click(this.id)\">R +5grad</button>\
                                  </body>\
                                  </html>");
}

void handleRootPOST() {
  Serial.println("Got POST request!");
  String data = server.arg("plain");
  Serial.println(data);
  /*StaticJsonDocument<200> jBuffer;
  //JsonObject& jObject = jBuffer.parseObject(data);
  deserializeJson(jBuffer, data);
  int id = jBuffer["id"];*/

  int id = data.toInt();

  if (id < 1 || id > 4) {
    server.send(503, "");
    return;
  }

  int step = 0;
  if (id % 2 == 1) {
    step = -SERVO_STEP;
  } else {
    step = SERVO_STEP;
  }

  if (id < 3) {
    double new_angle = max(min(servo_L_angle + step, 180.0), 0.0);
    servo_L.write(new_angle);
    servo_L_angle = new_angle;
  } else {
    double new_angle = max(min(servo_R_angle + step, 180.0), 0.0);
    servo_R.write(new_angle);
    servo_R_angle = new_angle;
  }
  
  server.send(204, "");
}
