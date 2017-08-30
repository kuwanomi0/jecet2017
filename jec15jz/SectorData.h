/*
 *　コースの区間ごとの設定パラメータを保持するクラス
 *  SectorData.h
 *
 */
class SectorData {
    int sectorId;       //  区間ID
    int judgeId;        //　判定方法ID（0:距離、1:時間、2:センサ）
    int distance;       //  判定用の距離
    int time;           //  判定用の時間
    int speed;          //  0-100   走行速度
    int turnControlId;  //  ターン制御ID(0:PID, 1:ダイレクト)
    float turn;           //      ダイレクト値
    float pidP;           //      P
    float pidI;           //      I
    float pidD;           //      D
    int ledColor;       //  LEDカラー　LED_ORANGE, LED_GREEN, LED_RED, LED_OFF

public:
    SectorData() {
    }
    SectorData(int sectorID, int judgeId, int distance, int time, int speed, int turnCotrolID,
            float turn, float pidP, float pidI, float pidD, float ledColor) {
        set(sectorID,  judgeId, distance, time, speed,  turnCotrolID,
             turn,  pidP,  pidI,  pidD,  ledColor);
    }
    void set(int sectorID, int judgeId, int distance, int time, int speed, int turnCotrolID,
            float turn, float pidP, float pidI, float pidD, float ledColor) {
        this->sectorId      = sectorId;
        this->judgeId       = judgeId;        //　判定方法ID（0:距離、1:時間、2:センサ）
        this->distance      = distance;       //  判定用の距離
        this->time          = time;           //  判定用の時間
        this->speed         = speed;          //  0-100
        this->turnControlId = turnControlId;  //  ターン制御ID(0:PID, 1:ダイレクト)
        this->turn          = turn;           //      ダイレクト値
        this->pidP          = pidP;           //      P
        this->pidI          = pidI;           //      I
        this->pidD          = pidD;           //      D
        this->ledColor      = ledColor;       // LEDカラー　LED_ORANGE, LED_GREEN, LED_RED, LED_OFF
    }
};
        
