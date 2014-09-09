/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Compass.h"

// don't allow any axis of the offset to go above 2000
#define COMPASS_OFS_LIMIT 2000

/*
 *  this offset learning algorithm is inspired by this paper from Bill Premerlani
 *
 *  http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
 *
 *  The base algorithm works well, but is quite sensitive to
 *  noise. After long discussions with Bill, the following changes were
 *  made:
 *
 *   1) we keep a history buffer that effectively divides the mag
 *      vectors into a set of N streams. The algorithm is run on the
 *      streams separately
 *
 *   2) within each stream we only calculate a change when the mag
 *      vector has changed by a significant amount.
 *
 *  This gives us the property that we learn quickly if there is no
 *  noise, but still learn correctly (and slowly) in the face of lots of
 *  noise.
 */
void
Compass::learn_offsets(void)
{
    if (_learn != 1) {
        // auto-calibration is disabled
        return;
    }

    // this gain is set so we converge on the offsets in about 5
    // minutes with a 10Hz compass
    const float gain = 0.01;
    const float max_change = 10.0;
    const float min_diff = 50.0;

    if (!_null_init_done) {
        // first time through
        _null_init_done = true;
        for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
            const Vector3f &ofs = _offset[k].get();
            for (uint8_t i=0; i<_mag_history_size; i++) {
                // fill the history buffer with the current mag vector,
                // with the offset removed
                _mag_history[k][i] = Vector3i((_field[k].x+0.5f) - ofs.x, (_field[k].y+0.5f) - ofs.y, (_field[k].z+0.5f) - ofs.z);
            }
            _mag_history_index[k] = 0;
        }
        return;
    }

    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        const Vector3f &ofs = _offset[k].get();
        Vector3f b1, diff;
        float length;

        if (ofs.is_nan()) {
            // offsets are bad possibly due to a past bug - zero them
            _offset[k].set(Vector3f());
        }

        // get a past element
        b1 = Vector3f(_mag_history[k][_mag_history_index[k]].x,
                      _mag_history[k][_mag_history_index[k]].y,
                      _mag_history[k][_mag_history_index[k]].z);

        // the history buffer doesn't have the offsets
        b1 += ofs;

        // get the current vector
        const Vector3f &b2 = _field[k];

        // calculate the delta for this sample
        diff = b2 - b1;
        length = diff.length();
        if (length < min_diff) {
            // the mag vector hasn't changed enough - we don't get
            // enough information from this vector to use it.
            // Note that we don't put the current vector into the mag
            // history here. We want to wait for a larger rotation to
            // build up before calculating an offset change, as accuracy
            // of the offset change is highly dependent on the size of the
            // rotation.
            _mag_history_index[k] = (_mag_history_index[k] + 1) % _mag_history_size;
            continue;
        }

        // put the vector in the history
        _mag_history[k][_mag_history_index[k]] = Vector3i((_field[k].x+0.5f) - ofs.x, 
                                                          (_field[k].y+0.5f) - ofs.y, 
                                                          (_field[k].z+0.5f) - ofs.z);
        _mag_history_index[k] = (_mag_history_index[k] + 1) % _mag_history_size;

        // equation 6 of Bills paper
        diff = diff * (gain * (b2.length() - b1.length()) / length);

        // limit the change from any one reading. This is to prevent
        // single crazy readings from throwing off the offsets for a long
        // time
        length = diff.length();
        if (length > max_change) {
            diff *= max_change / length;
        }

        Vector3f new_offsets = _offset[k].get() - diff;

        if (new_offsets.is_nan()) {
            // don't apply bad offsets
            continue;
        }

        // constrain offsets
        new_offsets.x = constrain_float(new_offsets.x, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
        new_offsets.y = constrain_float(new_offsets.y, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
        new_offsets.z = constrain_float(new_offsets.z, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
            
        // set the new offsets
        _offset[k].set(new_offsets);
    }
}

//GAF

void
Compass::calibrate(void)
{
	if(_learn != 3 && calibrate_flag !=3) {
		calibrate_led = 0;
		return;
	}
	calibrate_led = 3;
	
	for(uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) { 
		//Vector3f _delta_field[k] = _field[k] - _last_field[k];
		uint8_t cal_axis = 4;
		// decide which axis has the greatest reading and calibrate that axis.
		
		if (fabs(_field[k].x) > fabs(_field[k].y) && fabs(_field[k].x) > fabs(_field[k].z)) {
			//limit_set(_field[k].x, &_field_max[k].x, &_field_min[k].x, _delta_field[k].x);
			cal_axis = 1;
		}

		if (fabs(_field[k].y) > fabs(_field[k].z) && fabs(_field[k].y) > fabs(_field[k].x)) {
			//limit_set(_field[k].y, &_field_max[k].y, &_field_min[k].y, _delta_field[k].y);
			cal_axis = 2;
		}
		
		if (fabs(_field[k].z) > fabs(_field[k].y) && fabs(_field[k].z) > fabs(_field[k].x)) {
			//limit_set(_field[k].z, &_field_max[k].z, &_field_min[k].z, _delta_field[k].z);
			cal_axis = 3;
		}
			
		switch (cal_axis) {
			case 1:
				if(_field[k].x > _field_max[k].x) {
					_field_max[k].x = _field[k].x;
					//if (_delta_field[k].x > 0.01) {
					calibrate_led = 1;  // led is red
					//}
				}			
				else if (_field[k].x <= _field_max[k].x && _field[k].x > 0 ) {
					calibrate_led = 2;  // led is blue
				}
				//else calibrate_led = 3;
				if(_field[k].x < _field_min[k].x) {
					_field_min[k].x = _field[k].x;
					//if (_delta_field[k].x < 0.01) {
					calibrate_led = 1;  // led is red
					//}
				}			
				else if (_field[k].x >= _field_min[k].x && _field[k].x < 0 ) {
					calibrate_led = 2;  // led is blue
				}			
				
				//else calibrate_led = 3; 
			break;
			case 2:
				if(_field[k].y > _field_max[k].y) {
					_field_max[k].y = _field[k].y;
					//if (_delta_field[k].y > 0.01) {
					calibrate_led = 1;  // led is red
					//}
				}
				//else if (_field[k].y <= _field_max[k].y && _field[k].y > 0 && _delta_field[k].y < -0.2) {
				else if (_field[k].y <= _field_max[k].y && _field[k].y > 0 ) {
					calibrate_led = 2;  // led is blue
				}
				//else calibrate_led = 3;
				if(_field[k].y < _field_min[k].y) {
					_field_min[k].y = _field[k].y;
					//if (_delta_field[k].y < 0.01) {
					calibrate_led = 1;  // led is red
					//}
				}			
				else if (_field[k].y >= _field_min[k].y && _field[k].y < 0 ) {
					calibrate_led = 2;  // led is blue
				}			
				
				//else calibrate_led = 3;
			break;
			case 3:
				if(_field[k].z > _field_max[k].z) {
					_field_max[k].z = _field[k].z;
					//if (_delta_field[k].z > 0.01) {
					calibrate_led = 1;  // led is red
					//}
				}
				//else if (_field[k].z <= _field_max[k].z && _field[k].z > 0 && _delta_field[k].z < -0.2) {
				else if (_field[k].z <= _field_max[k].z && _field[k].z > 0 ) {
					calibrate_led = 2;  // led is blue
				}
				//else calibrate_led = 3;
				if(_field[k].z < _field_min[k].z) {
					_field_min[k].z = _field[k].z;
					//if (_delta_field[k].z < 0.01) {
					calibrate_led = 1;  // led is red
					//}
				}			
				else if (_field[k].z >= _field_min[k].z && _field[k].z < 0 ) {
					calibrate_led = 2;  // led is blue
				}			
				
				//else calibrate_led = 3;
			break;
			default :
				calibrate_led = 0;
			break;
		}
			
		//_last_field[k] = _field[k];
	}
}

/*
void
Compass::limit_set(float field, float *field_max, float *field_min, float delta_field)
{
	if(field > *field_max) {
		*field_max = field;
		if (delta_field > 0.2) {
			calibrate_led = 1;  // led is blue
			
		}
	}
	else if (field <= *field_max && field > 0 && delta_field < -0.2) {
		calibrate_led = 1;  // led is red
		
	}
	else calibrate_led = 0;
	
	if(field < *field_min) {
		*field_min = field;
		if (delta_field < -0.2) {
			calibrate_led = 1;  //led is blue
			
		}
	}
	else if (field >= *field_min && field < 0 && delta_field > 0.2) {
		calibrate_led = 1;  // led is red
		
	}
	else calibrate_led = 0;
}
*/
void
Compass::save_calibration(void)
{	
	float scale_XY = 1;
	float scale_XZ = 1;
	
	for(uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
		// Calculate the offsets and scale fators.
		_field_offset[k] = (_field_min[k] + _field_max[k]) / -2.0f;
		_field_span[k] = _field_max[k] - _field_min[k];		
		scale_XY = _field_span[k].x / _field_span[k].y; 
		scale_XZ = _field_span[k].x / _field_span[k].z;
		// Save the result.
		set_and_save_calibration(k, _field_offset[k], scale_XY, scale_XZ);	
		// Reset the vectors in case a redo of the cal is done, otherwise inly a reset of the board will zero them.
		_field_max[k] = set_vector(0,0,0,k);
		_field_min[k] = set_vector(0,0,0,k);
		_field_span[k] = set_vector(1,1,1,k);
	}
	calibrate_flag = 0;
	_learn.set_and_save(0);	
}

