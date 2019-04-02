/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;

/* *
 * Inverted list of KeyFrames, accessible by BoW.
 * For each word of the vocabulary, there is a list of keyframes that contain it.
 */

class KeyFrameDatabase
{
public:

	/* *
	 * Constructor that spans mvInvertedFile of the same size of vocabulary.
	 * Invoked only from the System builder.
	 */
    KeyFrameDatabase(ORBVocabulary &voc);
    KeyFrameDatabase() {;}

    /* *
     * Add a KeyFrame to the database.
     * It adds to the lists lists of keyframes of each BoW that has registered.
     * @param pKF KeyFrame
     */		
   void add(KeyFrame* pKF);

   void set_vocab(ORBVocabulary* pvoc);

    /* *
     * Delete a KeyFrame from the database.
     * Deletes it from each list of keyframes of each BoW that it has registered.
     * @param pKF KeyFrame
     */
   void erase(KeyFrame* pKF);

    /* *
     * Free your memory, invoked from Tracking :: Reset.
     */
   void clear();

	 /* *
	 * From a KeyFrame it tries to detect a loop.
	 *
     * @param pKF KeyFrame
     * @param minScore Minimum amount of matching BoW to find to consider that a KeyFrame is candidate to close the loop.
	 * @returns The KeyFrames candidates to close the loop, they will have to pass the pose test.
	 *
	 * Candidates returned are those that have a minimum amount of BoWs matching those of the KeyFrame argument.
	 *
	 * Only called from LoopClosing :: DetectLoop, only once per keyframe, always with new or recent keyframes.
	 */
   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

	 /* *
	 * Try to relocate from a box.
	 * @param F Current frame whose pose is to be located.
	 * @returns Vector of KeyFrames candidates for relocation, in which we found enough BoW matches.
	 * These candidates must then pass a pose test.
	 */
   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:
	/* *
	 * Vocabulary BoW.
	 */
  // Associated vocabulary
  ORBVocabulary* mpVoc;

	/* *
	* Each element of the vector corresponds to a word from the BoW vocabulary.
	* Each element consists of a list of keyframes containing that word BoW.
	* Thus, from the word Bow (the word is the index in this vector), you get a list of KeyFrames that
	* Observe singular points whose descriptors correspond to it.
	* Used for loop closure and relocation.
	*/
  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;

	friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
	{
		boost::serialization::split_member(ar, *this, version);
	}
		
	template<class Archive>
	void save(Archive & ar, const unsigned int version) const;
	

	template<class Archive>
	void load(Archive & ar, const unsigned int version);
};

} //namespace ORB_SLAM

#endif
