#ifndef COURSE_H
#define COURSE_H

class Course {

private:
    int course_num;
    int dis;
    int forward;
    float kp;
    float ki;
    float kd;

public:
    Course(int course_num, int dis, int forward, float kp, float ki, float kd) {
        setCourse(course_num, dis, forward, kp, ki, kd);
    }
    void setCourse(int course_num, int dis, int forward, float kp, float ki, float kd) {
        this->course_num = course_num;
        this->dis = dis;
        this->forward = forward;
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }
    int getCourse_num() {
        return course_num;
    }
    int getDis() {
        return dis;
    }
    int getForward() {
        return forward;
    }
    float getP() {
        return kp;
    }
    float getI() {
        return ki;
    }
    float getD() {
        return kd;
    }
};

#endif