
class PID {
public:
    float kp;
    float ki;
    float kd;
    float targetpoint;
    // ��һ�ε����
    float prevError;
    // ����
    float intergral;
    // ΢��
    float derivative;

    PID(float kp, float ki, float kd);

    void Set_PID(float kp, float ki, float kd);

    /**
     * pid�ļ��㺯��
     * @param target  Ŀ��ֵ
     * @param current ��ǰֵ
     * @return  pwm
     */
    float compute(float target, float current);

    /**
     *  �������е����: �����õ��ٶ� �� ��һ�β�һ��
     */
    void reset();
};
