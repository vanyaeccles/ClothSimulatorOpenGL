#pragma once
#ifndef _SWEEPANDPRUNE_H_
#define _SWEEPANDPRUNE_H_





class SweepAndPruneData
{

public:


	//Three arrays to store overlaps on each dimension

	/*
	'The overlap status consists of a boolean flag for each dimension.
	Whenever all three of these flags are set, the bounding boxes of the polytope pair overlap'
	*/
	bool overlapStatusX[NUM_RIGIDBODIES][NUM_RIGIDBODIES];

	bool overlapStatusY[NUM_RIGIDBODIES][NUM_RIGIDBODIES];

	bool overlapStatusZ[NUM_RIGIDBODIES][NUM_RIGIDBODIES];



	SweepAndPruneData()
	{
		//initialise all to false
		setPotColDim();
	}




	bool checkOverlapX(int i, int j)
	{
		return overlapStatusX[i][j];
	}
	bool checkOverlapY(int i, int j)
	{
		return overlapStatusY[i][j];
	}
	bool checkOverlapZ(int i, int j)
	{
		return overlapStatusZ[i][j];
	}


	void setPotColDim()
	{
		{
			for (int i = 0; i < NUM_RIGIDBODIES; i++)
				
				for (int j = 0; j < NUM_RIGIDBODIES; j++)
				{
					overlapStatusX[i][j] = false;
					overlapStatusY[i][j] = false;
					overlapStatusZ[i][j] = false;
				}
					
		}
	}


};

#endif