#include "track_psvr.h"

static float dist_3d(xrt_vec3 a, xrt_vec3 b)
{
	xrt_vec3 d;
	d.x = a.x - b.x;
	d.y = a.y - b.y;
	d.z = a.z - b.z;
	return sqrt(d.x*d.x + d.y*d.y + d.z * d.z);
}

bool psvr_disambiguate_5points(std::vector<psvr_led_t>* leds, psvr_track_data* t){
	//create a list of the corners, ignoring the center
	std::vector<uint32_t> corner_indices;
	for (uint32_t i=0;i<leds->size();i++)
	{
		psvr_led_t p = leds->at(i);
	  if (p.sign_x == 0 && p.sign_y == 0)
	  {
		  t->translation = p.position;
		  t->confidence[2] = i;
		  t->positions_3d[2] = p.position;
		  t->l_positions_2d[2] = p.l_position_2d;
		  t->r_positions_2d[2] = p.r_position_2d;
	  }
	  else
	  {corner_indices.push_back(i);}
	}

	//find the leftmost and rightmost points - these will belong to our left and right side edges.

	float lowest_x=65535.0;
	float highest_x=-65535.0;
	uint32_t lowest_x_index,highest_x_index;
	for (uint32_t i=0;i< corner_indices.size();i++)
	{
		psvr_led_t p = leds->at(corner_indices[i]);
		if (p.position.x < lowest_x)
		{
			lowest_x=p.position.x;
			lowest_x_index=corner_indices[i];
		}
		if (p.position.x > highest_x)
		{
			highest_x=p.position.x;
			highest_x_index=corner_indices[i];
		}
	}
	//printf("lowestX %f lowestXIndex %d highestX %f highestXIndex %d\n",lowestX,lowestXIndex,highestX,highestXIndex);
	//find the corresponding (closest) point on the 'short side' for the left and right extremities

	float lowest_l_x_distance = 65535.0f;
	float lowest_h_x_distance = 65535.0f;
	uint32_t lowest_x_pair_index;
	uint32_t highest_x_pair_index;

	psvr_led_t lcA = leds->at(lowest_x_index);
	for (uint32_t i=0; i < leds->size();i++)
	{
		psvr_led_t lcB = leds->at(corner_indices[i]);
		if  (corner_indices[i] != lowest_x_index)
		{
			float dist_l_x = dist_3d(lcA.position,lcB.position);
			if ( dist_l_x < lowest_l_x_distance)
			{
				lowest_x_pair_index = corner_indices[i];
				lowest_l_x_distance = dist_l_x;
			}
		}
	 }
	psvr_led_t hcA = leds->at(highest_x_index);
	for (uint32_t i=0; i < corner_indices.size();i++)
	{
		psvr_led_t hcB = leds->at(corner_indices[i]);
		if  (corner_indices[i] != highest_x_index)
		{
			float dist_h_x = dist_3d(hcA.position,hcB.position);
			if (dist_h_x < lowest_h_x_distance)
			{
				highest_x_pair_index = corner_indices[i];
				lowest_h_x_distance=dist_h_x;
			}
		}

	}
	//printf("lowestLXDistance %f lowestXPairIndex %d lowestHXDistance %f highestXPairIndex %d\n",lowestLXDistance,lowestXPairIndex,lowestHXDistance,highestXPairIndex);

	//now we have 4 points, and can know which 2 are left and which 2 are right.

	psvr_led_t lA = leds->at(lowest_x_index);
	psvr_led_t lB = leds->at(lowest_x_pair_index);
	if (lA.position.y < lB.position.y)
	{
		//lA is upper left and lB is lower left
		t->positions_3d[0] = lA.position;
		t->l_positions_2d[0]=lA.l_position_2d;
		t->r_positions_2d[0]=lA.r_position_2d;
		t->confidence[0] = 1;
		t->positions_3d[3] = lB.position;
		t->l_positions_2d[3]=lB.l_position_2d;
		t->r_positions_2d[3]=lB.r_position_2d;
		t->confidence[3] = 1;

	}
	else
	{
		//lA is lower left and lB is upper left
		t->positions_3d[0] = lB.position;
		t->l_positions_2d[0]=lB.l_position_2d;
		t->r_positions_2d[0]=lB.r_position_2d;
		t->confidence[0] = 1;

		t->positions_3d[3] = lA.position;
		t->l_positions_2d[3]=lA.l_position_2d;
		t->r_positions_2d[3]=lA.r_position_2d;
		t->confidence[3] = 1;

	}

	psvr_led_t hA = leds->at(highest_x_index);
	psvr_led_t hB = leds->at(highest_x_pair_index);
	if (hA.position.y < hB.position.y)
	{
		//hA is upper right and rB is lower right
		t->positions_3d[1] = hA.position;
		t->l_positions_2d[1]=hA.l_position_2d;
		t->r_positions_2d[1]=hA.r_position_2d;

		t->confidence[1] = 1;
		t->positions_3d[4] = hB.position;
		t->l_positions_2d[4]=hB.l_position_2d;
		t->r_positions_2d[4]=hB.r_position_2d;
		t->confidence[4] = 1;
	}
	else
	{
		//hA is lower right and hB is upper right
		t->positions_3d[1] = hB.position;
		t->l_positions_2d[1]=hB.l_position_2d;
		t->r_positions_2d[1]=hB.r_position_2d;
		t->confidence[1] = 1;
		t->positions_3d[4] = hA.position;
		t->l_positions_2d[4]=hA.l_position_2d;
		t->r_positions_2d[4]=hA.l_position_2d;
		t->confidence[4] = 1;
	}
	return true;
}

/*//TODO: we dont need to pass a TrackData* here
bool psvr_compute_svd()
{
	//compute SVD for the points we have found, assuming we have at least 3 points

	uint8_t pointCount=0;
	for (uint32_t i=0;i<MAX_POINTS;i++)
	{
		if (t->confidence[i] > 0)
		{
			pointCount++;
		}
	}

	if (pointCount > 2)
	{
		cv::Mat measurement(pointCount, 3, cv::DataType<float>::type);
		cv::Mat model(pointCount, 3, cv::DataType<float>::type);
		cv::Mat xCovar;
		uint8_t c = 0;
		for (uint32_t i=0;i<MAX_POINTS;i++)
		{
			if (t->confidence[i] > 0)
			{
				measurement.at<float>(c,0) = t->positions[i].x;
				measurement.at<float>(c,1) = t->positions[i].y;
				measurement.at<float>(c,2) = t->positions[i].z;
				model.at<float>(c,0) = ledPositions[i].x;
				model.at<float>(c,1) = ledPositions[i].y;
				model.at<float>(c,2) = ledPositions[i].z;
				c++;
			}
		}

		// create our cross-covariance matrix
		cv::transpose(model,model);
		xCovar =  model * measurement;
		cv::Mat w,u,v,ut;
		decomposer->compute(xCovar,w,u,v);
		cv::transpose(u,ut);
		//TODO: compute determinant
		cv::Mat rot = v * ut;
		glm::mat3 glmRot;
		memcpy((void*)&(glmRot[0][0]),rot.data, 9 * sizeof(float));
		glm::mat3 tRot = glm::transpose(glmRot);
		t->rotationMatrix = glm::mat4(tRot);
		//cout << "R = "<< endl << " "  << rotationMatrix << endl << endl;
		return true;
	}
	else
	{
		return false;
	}
}*/

