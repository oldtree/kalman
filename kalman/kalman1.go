package kalman

/*
 * NOTES: n Dimension means the state is n dimension,
 * measurement always 1 dimension
 */

/* 1 Dimension */

type OneDimensionKalman struct {
	State float64 //state

	A float64 //x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
	H float64 //z(n)=H*x(n)+w(n),w(n)~N(0,r)

	Q float64 //process(predict) noise convariance 预测协方差
	R float64 //measure noise convariance 噪音协方差

	P    float64 //estimated error convariance 估计错误协方差
	Gain float64 //增益
}

/*
 * @brief
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1;
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs
 *   state - Klaman filter structure
 *   init_x - initial x state value
 *   init_p - initial estimated error convariance
 * @outputs
 * @retval
 */
func (o *OneDimensionKalman) Init(init_state float64, p_measure float64) {
	o.State = init_state
	o.P = p_measure
	o.A = 1
	o.H = 1
	o.R = 10e-6
	o.Q = 10e-6
	o.Gain = 10e-6
}

/*
 * @brief
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0};
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs
 * @outputs
 * @retval
 */

func (o *OneDimensionKalman) KalmanFilter(measure float64) {
	/* Predict */
	o.State = o.State * o.A
	o.P = o.A*o.A*o.P + o.Q ///* p(n|n-1)=A^2*p(n-1|n-1)+q */
	/* Measurement */
	o.Gain = o.P * o.H / (o.P*o.H*o.H + o.R)
	o.State = o.State + o.Gain*(measure-o.H*o.State)

	o.P = (1 - o.Gain*o.H) * o.P
}

type TwoDimensionKalman struct {
	State [2]float64    //state
	A     [2][2]float64 //x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
	H     [2]float64    //z(n)=H*x(n)+w(n),w(n)~N(0,r)
	Q     [2]float64    //process(predict) noise convariance 预测协方差
	R     float64       //measure noise convariance 噪音协方差
	P     [2][2]float64 //estimated error convariance 估计错误协方差
	Gain  [2]float64    //增益
}

/*
 * @brief
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0};
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs
 * @outputs
 * @retval
 */

func (t *TwoDimensionKalman) Init(initStats []float64, p_measure [][]float64) {
	t.State = [2]float64{0.0, 0.0}
	t.A = [2][2]float64{{0.0, 0.0}, {0.0, 0.0}}
	t.H = [2]float64{0.0, 0.0}
	t.Q = [2]float64{0.0, 0.0}
	t.R = 0.0
	t.P = [2][2]float64{{0.0, 0.0}, {0.0, 0.0}}
	t.Gain = [2]float64{0.0, 0.0}

	t.State[0] = initStats[0]
	t.State[1] = initStats[1]

	t.P[0][0] = p_measure[0][0]
	t.P[0][1] = p_measure[0][1]
	t.P[1][0] = p_measure[1][0]
	t.P[1][1] = p_measure[1][1]

	t.A[0][0] = 1
	t.A[0][1] = 0.1
	t.A[1][0] = 0
	t.A[1][1] = 1

	t.H[0] = 1
	t.H[1] = 0

	t.Q[0] = 10e-7
	t.Q[1] = 10e-7

	t.R = 10e-7

}

/*
 * @brief
 *   2 Dimension kalman filter
 * @inputs
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs
 *   t.State[0] - Updated state value, Such as angle,velocity
 *   t.State[1] - Updated state value, Such as diffrence angle, acceleration
 *   t.P    - Updated estimated error convatiance matrix
 * @retval
 *   Return value is equals to t.State[0], so maybe angle or velocity.
 */

func (t *TwoDimensionKalman) KalmanFilter(z_measure float64) {
	var temp0, temp1, temp2 float64
	temp0, temp1, temp2 = 0.0, 0.0, 0.0
	/* Step1: Predict */
	t.State[0] = t.A[0][0]*t.State[0] + t.A[0][1]*t.State[0]
	t.State[1] = t.A[1][0]*t.State[0] + t.A[1][1]*t.State[1]
	/* p(n|n-1)=A^2*p(n-1|n-1)+q */
	t.P[0][0] = t.A[0][0]*t.P[0][0] + t.A[0][1]*t.P[1][0] + t.Q[0]
	t.P[0][1] = t.A[0][0]*t.P[0][1] + t.A[1][1]*t.P[1][1]
	t.P[1][0] = t.A[1][0]*t.P[0][0] + t.A[0][1]*t.P[1][0]
	t.P[1][1] = t.A[1][0]*t.P[0][1] + t.A[1][1]*t.P[1][1] + t.Q[1]
	/* Step2: Measurement */
	/* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
	temp0 = t.P[0][0]*t.H[0] + t.P[0][1]*t.H[1]
	temp1 = t.P[1][0]*t.H[0] + t.P[1][1]*t.H[1]
	temp2 = t.R + t.H[0]*temp0 + t.H[1]*temp1
	t.Gain[0] = temp0 / temp2
	t.Gain[1] = temp1 / temp2
	/* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
	temp2 = t.H[0]*t.State[0] + t.H[1]*t.State[1]
	t.State[0] = t.State[0] + t.Gain[0]*(z_measure-temp2)
	t.State[1] = t.State[1] + t.Gain[1]*(z_measure-temp2)

	/* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
	t.P[0][0] = (1 - t.Gain[0]*t.H[0]) * t.P[0][0]
	t.P[0][1] = (1 - t.Gain[0]*t.H[1]) * t.P[0][1]
	t.P[1][0] = (1 - t.Gain[1]*t.H[0]) * t.P[1][0]
	t.P[1][1] = (1 - t.Gain[1]*t.H[1]) * t.P[1][1]
}
