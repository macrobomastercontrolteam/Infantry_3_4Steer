#include "user_lib.h"
#include "arm_math.h"

fp32 first_order_filter(fp32 input, fp32 output_prev, fp32 coeff)
{
    if (coeff > 1)
    {
        return NAN;
    }
    fp32 output = (1-coeff)*output_prev + coeff*input;
    return output;
}

fp32 second_order_filter(fp32 input, fp32 output_prev1, fp32 output_prev2, fp32 coeff1, fp32 coeff2)
{
    if (coeff1 + coeff2 >= 1)
    {
        return NAN;
    }
    fp32 output = coeff1*input + coeff2*output_prev1 + (1-coeff1-coeff2)*output_prev2;
    return output;
}

fp32 brakezone(fp32 input, fp32 threshold, fp32 order)
{
	// tuning techniques:
	// adjustment step: threshold, then order
	// threshold: affects oscillation amplitude and curvature around 0
	fp32 input_abs = fabs(input);
	if ((threshold <= 0) || (order <= 0))
	{
		// should not reach here
		// disabled case
		return input;
	}
	else if (input_abs >= threshold)
	{
		return input;
	}
	else
	{
		return input * pow(input_abs / threshold, order);
	}
}

fp32 brakezone_symmetric(fp32 input, fp32 threshold, fp32 order)
{
	fp32 output = input;
	if ((threshold <= 0) || (order <= 0))
	{
		// should not reach here
		// disabled case
	}
	else
	{
		fp32 input_abs = fabs(input);
		if (input_abs < threshold)
		{
			output = input * pow(input_abs / threshold, order);
		}
		else
		{
			fp32 twice_threshold = 2.0f * threshold;
			if (input_abs <= twice_threshold)
			{
				fp32 x_abs_minus_2s = input_abs - twice_threshold;
				output = SIGN(input) * (x_abs_minus_2s * pow(fabs(x_abs_minus_2s) / threshold, order) + twice_threshold);
			}
			else
			{
				// bypass
			}
		}
	}
	return output;
}

//���ٿ���
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          б��������ʼ��
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      ���ֵ
  * @param[in]      ��Сֵ
  * @retval         ���ؿ�
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
  * @brief          Moving average
  * @author         2022 MacFalcons
  * @param[in]      input
  * @param[in]      handler
  * @retval         average
  */
fp32 moving_average_calc(fp32 input, moving_average_type_t* moving_average_type, uint8_t fInit)
{
    fp32 output;
    if (fInit == MOVING_AVERAGE_RESET)
    {
        moving_average_type->sum = input * moving_average_type->size;
        for (uint8_t i = 0; i < (moving_average_type->size); i++)
        {
            moving_average_type->ring[i] = input;
        }
        moving_average_type->cursor = 0;
        output = input;
    }
    else
    {
        // history[cursor] is the current oldest history in the ring
        moving_average_type->sum = moving_average_type->sum - moving_average_type->ring[moving_average_type->cursor] + input;
        moving_average_type->ring[moving_average_type->cursor] = input;
        moving_average_type->cursor = (moving_average_type->cursor + 1) % moving_average_type->size;
        output = (moving_average_type->sum) / ((fp32)(moving_average_type->size));
    }
    return output;
}

//��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//�޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//���ȸ�ʽ��Ϊ-PI~PI

//�Ƕȸ�ʽ��Ϊ-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

uint8_t matrixMultiplication(uint8_t m1_rows, uint8_t m1_cols, uint8_t m2_rows, uint8_t m2_cols, fp32 m1[m1_rows][m1_cols], fp32 m2[m2_rows][m2_cols], fp32 result[m1_rows][m2_cols])
{
    uint8_t fvalid;
    fvalid = (m1_cols == m2_rows);
    fvalid &= ((m1 != result) && (m2 != result));
    if (fvalid)
    {
        // printf("Resultant Matrix is:\n");
        for (uint8_t i = 0; i < m1_rows; i++)
        {
            for (uint8_t j = 0; j < m2_cols; j++)
            {
                result[i][j] = 0;

                for (uint8_t k = 0; k < m2_rows; k++)
                {
                    result[i][j] += m1[i][k] * m2[k][j];
                }
                // printf("%f\t", result[i][j]);
            }
            // printf("\n");
        }
    }
    return fvalid;
}

void LimitMax(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}
