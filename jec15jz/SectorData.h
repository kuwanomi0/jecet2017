/*
 *�@�R�[�X�̋�Ԃ��Ƃ̐ݒ�p�����[�^��ێ�����N���X
 *  SectorData.h
 *
 */
class SectorData {
    int sectorId;       //  ���ID
    int judgeId;        //�@������@ID�i0:�����A1:���ԁA2:�Z���T�j
    int distance;       //  ����p�̋���
    int time;           //  ����p�̎���
    int speed;          //  0-100   ���s���x
    int turnControlId;  //  �^�[������ID(0:PID, 1:�_�C���N�g)
    float turn;           //      �_�C���N�g�l
    float pidP;           //      P
    float pidI;           //      I
    float pidD;           //      D
    int ledColor;       //  LED�J���[�@LED_ORANGE, LED_GREEN, LED_RED, LED_OFF

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
        this->judgeId       = judgeId;        //�@������@ID�i0:�����A1:���ԁA2:�Z���T�j
        this->distance      = distance;       //  ����p�̋���
        this->time          = time;           //  ����p�̎���
        this->speed         = speed;          //  0-100
        this->turnControlId = turnControlId;  //  �^�[������ID(0:PID, 1:�_�C���N�g)
        this->turn          = turn;           //      �_�C���N�g�l
        this->pidP          = pidP;           //      P
        this->pidI          = pidI;           //      I
        this->pidD          = pidD;           //      D
        this->ledColor      = ledColor;       // LED�J���[�@LED_ORANGE, LED_GREEN, LED_RED, LED_OFF
    }
};
        
