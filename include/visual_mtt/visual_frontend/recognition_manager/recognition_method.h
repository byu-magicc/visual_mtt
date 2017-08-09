#pragma once

namespace visual_frontend {

  class RecognitionMethod
  {
  public:
    // This receives normalized image plane coordinates of a recently-elevated
    // good model. A high resolution image of the object is cropped and used to
    // compare to visual information of historical tracks. It returns an ID
    // number (GMN). A return of 0 means no match was made to previous tracks.
    virtual uint32_t compare_history(double x, double y) = 0;

    // Another function here for updating the historical information

    // Functions for updating camera info, storing hd image, etc

  };

}
