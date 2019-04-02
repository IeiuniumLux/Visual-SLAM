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


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{

/* * Vocabulary that maps ORB descriptors with Bow.
 * OBRVocabulary is a template-based type that defines a type of descriptors and a class of DBOW2 functions to manipulate them.
 * OBRVocabulary uses Mat as descriptor (TDescriptor is Mat), and the FORB class as implementation specific for ORB
 * Of the general descriptor manipulation functions required by DBOW2.
 *
 * Main.cc creates the only instance of this object, loads its vocabulary from a file,
 * And then passes it as a reference to the main constructors, starting the cascade.
 * In the future, every object that needs a vocabulary receives it by reference when it is built.
 *
 * Frame, KeyFrame, LoopClosing and Tracking call it mpORBvocabulary.
 * KeyFrameDatabase calls it mpVoc.
 * Frame and KeyFrame use only their transform method, which obtains BoW corresponding to a set of descriptors.
 * LoopClosing uses only its score method, which compares two BoWs.
 * KeyFrameDatabase uses score and size, the latter simply reports the number of words in the vocabulary.
* Tracking is limited to passing it to the objects it builds. It acts as handrails.
 *
 *
 *
 *
 */

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
