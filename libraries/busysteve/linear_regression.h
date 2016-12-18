


class linear_regression
{
	double *_tss;   // timestamp or 'x'
	double *_lreg;  // data samples or 'y' 

	long _counter;
	long _samples;

	double _m;
	double _b;

	public:

	linear_regression( int samples )
		: _samples(samples), _counter(0), _lreg(NULL), _tss(NULL)
	{
		_tss  = new double[_samples];
		_lreg = new double[_samples];
	}

	void log_entry( double x, double y )
	{
		_lreg[_counter%_samples] = y;
		_tss[_counter%_samples]  = x;
		_counter++;
	}

	double avg()
	{
		double tot = 0.0;

		int len = _counter < _samples ? _counter : _samples;

		for( int i = 0; i < len; i++ )
		{
			tot += _lreg[i];
		}
		
		return tot / (double)len;
	}

	double mid_avg()
	{
		double tot = 0.0;
		double min = _lreg[0];
		double max = 0.0;

		int len = _counter < _samples ? _counter : _samples;

		for( int i = 0; i < len; i++ )
		{
			tot += _lreg[i];

			min = min > _lreg[i] ? _lreg[i] : min;
			max = max < _lreg[i] ? _lreg[i] : max;
		}
		
		tot -= min;
		tot -= max;

		return tot / (double)(len-2);
	}

	double stddev()
	{
		double tot = 0.0;

		int len = _counter < _samples ? _counter : _samples;

		for( int i = 0; i < len; i++ )
		{
			tot += _lreg[i]*_lreg[i];
		}
		
		return sqrt(tot / (double)len);
	}

	double calc()
	{
		double lxy = 0.0;
		double lxx = 0.0;
		double lxs = 0.0;
		double lys = 0.0;

		int len = _counter < _samples ? _counter : _samples;

		for( int i = 0; i < len; i++ )
		{
			lxy += _lreg[i]*_tss[i];
			lxx += _tss[i]*_tss[i];
			lxs += _tss[i];
			lys += _lreg[i];
		}

		if( (((double)len*lxx)-(lxs*lxs)) == 0.0 )
			return _lreg[(_counter-1)%_samples];

		_m = (((double)len*lxy)-(lxs*lys))/(((double)len*lxx)-(lxs*lxs));
		_b = (lys-(_m*lxs))/len;

		return _m*(_tss[(_counter-1)%_samples])+_b;
	}

	double slope()
	{
		long cntr = (_counter-1);
		long cnt_min = ((cntr+1) - _samples) < 0 ? 0 : (cntr+1 - _samples); 
		long cnt_max = cntr; 
		cnt_min %= _samples;
		cnt_max %= _samples;
		double x1 = _tss[cnt_min];
		double x2 = _tss[cnt_max];
		double y1 = _m*(_tss[(cnt_min)])+_b;
		double y2 = _m*(_tss[(cnt_max)])+_b;
		
		if( (x2-x1) == 0.0 )
			return 0.0;

		double slp = (y2-y1) / (x2-x1);
		return slp;
	}

	double calc_center()
	{
		long cntr = (_counter-1);
		long cnt_min = ((cntr+1) - _samples) < 0 ? 0 : (cntr+1 - _samples); 
		long cnt_max = cntr; 
		cnt_min %= _samples;
		cnt_max %= _samples;
		double x1 = _tss[cnt_min];
		double x2 = _tss[cnt_max];
		double y1 = _m*(_tss[(cnt_min)])+_b;
		double y2 = _m*(_tss[(cnt_max)])+_b;
		
		if( (x2-x1) == 0.0 )
			return 0.0;

		double slp = (y2-y1) / (x2-x1);

		double center_slp = -slp * ( ( x2-x1 ) / 2.0 ) + y1;
		return center_slp;
	}

	~linear_regression()
	{
		delete [] _tss;
		delete [] _lreg;
	}

};


