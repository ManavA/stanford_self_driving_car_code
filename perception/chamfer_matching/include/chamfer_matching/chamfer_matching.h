/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Marius Muja

#ifndef CHAMFER_MATCHING_H_
#define CHAMFER_MATCHING_H_

#include <vector>

using namespace std;

typedef pair<int,int> coordinate_t;
typedef float orientation_t;;
typedef vector<coordinate_t> template_coords_t;
typedef vector<orientation_t> template_orientations_t;



/**
 * Finds a contour in an edge image. The original image is altered by removing the found contour.
 * @param templ_img Edge image
 * @param coords Coordinates forming the contour.
 * @return True while a contour is still found in the image.
 */
bool findContour(IplImage* templ_img, template_coords_t& coords);


/**
* Computes contour points orientations using the approach from:
*
* Matas, Shao and Kittler - Estimation of Curvature and Tangent Direction by
* Median Filtered Differencing
*
* @param coords Contour points
* @param orientations Contour points orientations
*/
void findContourOrientations(const template_coords_t& coords, template_orientations_t& orientations);


///////////////////////// Image iterators ////////////////////////////


typedef pair<CvPoint, float> location_scale_t;


class ImageIterator
{
public:
	virtual bool hasNext() const = 0;
	virtual location_scale_t next() = 0;
};

class ImageRange
{
public:
	virtual ImageIterator* iterator() const = 0;
};


// Sliding window

class SlidingWindowImageRange : public ImageRange
{
	int width_;
	int height_;
	int x_step_;
	int y_step_;
	int scales_;
	float min_scale_;
	float max_scale_;

public:
	SlidingWindowImageRange(int width, int height, int x_step = 3, int y_step = 3, int scales = 5, float min_scale = 0.6, float max_scale = 1.6) :
		width_(width), height_(height), x_step_(x_step),y_step_(y_step), scales_(scales), min_scale_(min_scale), max_scale_(max_scale)
	{
	}

	ImageIterator* iterator() const;
};

class LocationImageRange : public ImageRange
{
	const vector<CvPoint>& locations_;

	int scales_;
	float min_scale_;
	float max_scale_;

public:
	LocationImageRange(const vector<CvPoint>& locations, int scales = 5, float min_scale = 0.6, float max_scale = 1.6) :
		locations_(locations), scales_(scales), min_scale_(min_scale), max_scale_(max_scale)
	{
	}

	ImageIterator* iterator() const;
};


class LocationScaleImageRange : public ImageRange
{
	const vector<CvPoint>& locations_;
	const vector<float>& scales_;

public:
	LocationScaleImageRange(const vector<CvPoint>& locations, const vector<float>& scales) :
		locations_(locations), scales_(scales)
	{
		assert(locations.size()==scales.size());
	}

	ImageIterator* iterator() const;
};




/**
 * Class that represents a template for chamfer matching.
 */
class ChamferTemplate
{
	vector<ChamferTemplate*> scaled_templates;
	vector<int> addr;
	int addr_width;
public:
	template_coords_t coords;
	template_orientations_t orientations;
	CvSize size;
	CvPoint center;
	float scale;

	ChamferTemplate() : addr_width(-1)
	{
	}

	ChamferTemplate(IplImage* edge_image, float scale_ = 1);

	~ChamferTemplate()
	{
		for (size_t i=0;i<scaled_templates.size();++i) {
			delete scaled_templates[i];
		}
	}

	vector<int>& getTemplateAddresses(int width);

	/**
	 * Resizes a template
	 *
	 * @param scale Scale to be resized to
	 */
	ChamferTemplate* rescale(float scale);

	void show() const;

};



//const int MAX_MATCHES = 20;

/**
 * Used to represent a matching result.
 */
class ChamferMatch
{

	int max_matches_;
	float min_match_distance_;

public:


	class ChamferMatchInstance {
	public:
		float cost;
		CvPoint offset;
		const ChamferTemplate* tpl;
//		vector<float> costs;
//		vector<int> img_offs;
	};

	typedef vector<ChamferMatchInstance> ChamferMatches;

	int count;
	ChamferMatches matches;


	ChamferMatch(int max_matches = 20, float min_match_distance = 10.0) :  max_matches_(max_matches),
		min_match_distance_(min_match_distance), count(0)
	{
		matches.resize(max_matches_);
	}


	void showMatch(IplImage* img, int index = 0);

	const ChamferMatches& getMatches() const { return matches; }

private:
	void addMatch(float cost, CvPoint offset, ChamferTemplate* tpl, const vector<int>& addr, const vector<float>& costs);

	friend class ChamferMatching;
};





/**
 * Implements the chamfer matching algorithm on images taking into account both distance from
 * the template pixels to the nearest pixels and orientation alignment between template and image
 * contours.
 */
class ChamferMatching
{
	float truncate_;
	bool use_orientation_;

	vector<ChamferTemplate*> templates;
public:
	ChamferMatching(bool use_orientation = true, float truncate = 20) : truncate_(truncate), use_orientation_(use_orientation)
	{
	}

	~ChamferMatching()
	{
		for (size_t i = 0; i<templates.size(); i++) {
			delete templates[i];
		}
	}

	/**
	 * Add a template to the detector from an edge image.
	 * @param templ An edge image
	 */
	void addTemplateFromImage(IplImage* templ, float scale = 1.0);

	/**
	 * Run matching using an edge image.
	 * @param edge_img Edge image
	 * @return a match object
	 */
	ChamferMatch matchEdgeImage(IplImage* edge_img, const ImageRange& range, float orientation_weight = 0.5, int max_matches = 20, float min_match_distance = 10.0);


private:

    /**
     * Computes the chamfer matching cost for one position in the target image.
     * @param dist_img Distance transform image.
     * @param orientation_img Orientation image.
     * @param templ_addr Offsets of the template points into the target image (used to speedup the search).
     * @param templ_orientations Orientations of the target points.
     * @param offset Offset where to compute cost
     * @param alpha Weighting between distance cost and orientation cost.
     * @return Chamfer matching cost
     */
    void localChamferDistance(CvPoint offset, IplImage* dist_img, IplImage* orientation_img, ChamferTemplate* tpl, ChamferMatch& cm, float orientation_weight);

    /**
     * Matches all templates.
     * @param dist_img Distance transform image.
     * @param orientation_img Orientation image.
     * @param cm Matching result
     */
    void matchTemplates(IplImage* dist_img, IplImage* orientation_img, ChamferMatch& cm, const ImageRange& range, float orientation_weight);


};



#endif /* CHAMFER_MATCHING_H_ */
