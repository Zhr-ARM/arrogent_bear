#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "PID.h"

/* �ڲ����ܺ��� */
static void pid_formula_incremental(PID_T * _tpPID);
static void pid_formula_positional(PID_T * _tpPID);
static void pid_out_limit(PID_T * _tpPID);

/*******************************************************************************
 * @brief PID��ʼ�����������ڳ�ʼ��PID�ṹ��
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @param {float} _kp ����ϵ��
 * @param {float} _ki ����ϵ��
 * @param {float} _kd ΢��ϵ��
 * @param {float} _target Ŀ��ֵ
 * @param {float} _limit ����޷�ֵ
 * @return {*}
 * @note ʹ��ǰ������øú�����ʼ��PID����
 *******************************************************************************/
void pid_init(PID_T * _tpPID, float _kp, float _ki, float _kd, float _target, float _limit)
{
    _tpPID->kp = _kp;          // ����
    _tpPID->ki = _ki;          // ����
    _tpPID->kd = _kd;          // ΢��
    _tpPID->target = _target;  // Ŀ��ֵ
    _tpPID->limit = _limit;    // �޷�ֵ
    _tpPID->integral = 0;      // ����������
    _tpPID->last_error = 0;    // �ϴ��������
    _tpPID->last2_error = 0;   // ���ϴ��������
    _tpPID->out = 0;           // ���ֵ����
    _tpPID->p_out = 0;         // P�������
    _tpPID->i_out = 0;         // I�������
    _tpPID->d_out = 0;         // D�������
}

/*******************************************************************************
 * @brief ����PIDĿ��ֵ
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @param {float} _target Ŀ��ֵ
 * @return {*}
 * @note ���ڶ�̬����PID��������Ŀ��ֵ
 *******************************************************************************/
void pid_set_target(PID_T * _tpPID, float _target)
{
    _tpPID->target = _target;
    pid_init(_tpPID,
           _tpPID->kp, _tpPID->ki, _tpPID->kd,
           _tpPID->target, _tpPID->limit);
}

/*******************************************************************************
 * @brief ����PID����
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @param {float} _kp ����ϵ��
 * @param {float} _ki ����ϵ��
 * @param {float} _kd ΢��ϵ��
 * @return {*}
 * @note ���ڶ�̬����PID����
 *******************************************************************************/
void pid_set_params(PID_T * _tpPID, float _kp, float _ki, float _kd)
{
    _tpPID->kp = _kp;
    _tpPID->ki = _ki;
    _tpPID->kd = _kd;
}

/*******************************************************************************
 * @brief ����PID����޷�
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @param {float} _limit �޷�ֵ
 * @return {*}
 * @note ���ڶ�̬����PID����޷�
 *******************************************************************************/
void pid_set_limit(PID_T * _tpPID, float _limit)
{
    _tpPID->limit = _limit;
}

/*******************************************************************************
 * @brief ����PID������
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @return {*}
 * @note ���������ʷ�������
 *******************************************************************************/
void pid_reset(PID_T * _tpPID)
{
    _tpPID->integral = 0;
    _tpPID->last_error = 0;
    _tpPID->last2_error = 0;
    _tpPID->out = 0;
    _tpPID->p_out = 0;
    _tpPID->i_out = 0;
    _tpPID->d_out = 0;
}

/*******************************************************************************
 * @brief ����λ��ʽPID
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @param {float} _current ��ǰֵ
 * @return {float} PID���������ֵ
 * @note λ��ʽPID��P-��Ӧ�ԣ�I-׼ȷ�ԣ�D-�ȶ���
 *******************************************************************************/
float pid_calculate_positional(PID_T * _tpPID, float _current)
{
    _tpPID->current = _current;
    pid_formula_positional(_tpPID);
    pid_out_limit(_tpPID);
    return _tpPID->out;
}

/*******************************************************************************
 * @brief ��������ʽPID
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @param {float} _current ��ǰֵ
 * @return {float} PID���������ֵ
 * @note ����ʽPID��P-�ȶ��ԣ�I-��Ӧ�ԣ�D-׼ȷ��
 *******************************************************************************/
float pid_calculate_incremental(PID_T * _tpPID, float _current)
{
    _tpPID->current = _current;
    pid_formula_incremental(_tpPID);
    pid_out_limit(_tpPID);
    return _tpPID->out;
}

/* ������������������������������������������������������������������ PID��صĹ��ܺ��� ������������������������������������������������������������������ */
/*******************************************************************************
 * @brief ����޷�����
 * @param {PID_T *} _tpPID ָ��PID�ṹ���ָ��
 * @return {*}
 * @note ��ֹ��������޶���Χ
 *******************************************************************************/
static void pid_out_limit(PID_T * _tpPID)
{
    if(_tpPID->out > _tpPID->limit)
        _tpPID->out = _tpPID->limit;
    else if(_tpPID->out < -_tpPID->limit)
        _tpPID->out = -_tpPID->limit;
}

/*******************************************************************************
 * @brief ����ʽPID��ʽ
 * @param {PID_T *} _tpPID  ����Ҫ�����PID����ָ��
 * @return {*}
 * @note ������ʽ�У�P-�ȶ��ԣ�I-��Ӧ�ԣ�D-׼ȷ��
 *******************************************************************************/
static void pid_formula_incremental(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;
    
    _tpPID->p_out = _tpPID->kp * (_tpPID->error - _tpPID->last_error);
    _tpPID->i_out = _tpPID->ki * _tpPID->error;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - 2 * _tpPID->last_error + _tpPID->last2_error);
    
    _tpPID->out += _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;
    
    _tpPID->last2_error = _tpPID->last_error;
    _tpPID->last_error = _tpPID->error;
}

/*******************************************************************************
 * @brief λ��ʽPID��ʽ
 * @param {PID_T *} _tpPID  ����Ҫ�����PID����ָ��
 * @return {*}
 * @note ��λ��ʽ�У�P-��Ӧ�ԣ�I-׼ȷ�ԣ�D-�ȶ���
 *******************************************************************************/
static void pid_formula_positional(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;
    
    // 积分分离：误差大时不使用积分项，减少震荡
    if(fabs(_tpPID->error) > 10.0f) {
        _tpPID->integral = 0; // 大误差时清零积分
    } else {
        _tpPID->integral += _tpPID->error;
        // 积分限幅，防止积分饱和，适中的限制
        if(_tpPID->integral > 100.0f) _tpPID->integral = 100.0f;
        if(_tpPID->integral < -100.0f) _tpPID->integral = -100.0f;
    }
    
    _tpPID->p_out = _tpPID->kp * _tpPID->error;
    _tpPID->i_out = _tpPID->ki * _tpPID->integral;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - _tpPID->last_error);
    
    _tpPID->out = _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;
    
    _tpPID->last_error = _tpPID->error;
} 

/**
 * @brief �޷�����
 * @param value ����ֵ
 * @param min ��Сֵ
 * @param max ���ֵ
 * @return �޷����ֵ
 */
float pid_constrain(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

/**
 * @brief �����޷�����
 * @param pid PID������
 * @param min ��Сֵ
 * @param max ���ֵ
 * @note ��ʹ������ʽPID����ʱ�˺�������Ҫ����Ϊ�˿����л���λ��ʽPID������
 */
void __attribute__((unused)) pid_app_limit_integral(PID_T *pid, float min, float max)
{
    if (pid->integral > max)
    {
        pid->integral = max;
    }
    else if (pid->integral < min)
    {
        pid->integral = min;
    }
}

// pid.c中新增
void pid_update_target(PID_T * _tpPID, float _target) {
    _tpPID->target = _target;
    // 不调用pid_init,保留积分等状态
}
