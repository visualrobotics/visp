//! \example tutorial-hand-eye-calibration.cpp
#include <visp3/vision/vpHandEyeCalibration.h>

int main(int argc, char *argv[])
{
  unsigned int ndata = 0;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ndata" && i+1 < argc) {
      ndata = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--help") {
      std::cout << argv[0] << " [--ndata <number of data to process>] "
                              "[--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }
  if (ndata == 0) {
    std::cout << "Number of data to process not specified" << std::endl;
    std::cout << argv[0] << " --help" << std::endl;
    return EXIT_SUCCESS;
  }
  std::vector<vpHomogeneousMatrix> cMo(ndata);
  std::vector<vpHomogeneousMatrix> wMe(ndata);
  vpHomogeneousMatrix eMc;

  vpPoseVector start(0,0.1,0,3.1415,0,0);
  vpPoseVector rel(0.1,0.1,0.1,2.22,2.22,0);

  vpHomogeneousMatrix start_h = vpHomogeneousMatrix(start);
  vpHomogeneousMatrix rel_h =  vpHomogeneousMatrix(rel);
  vpHomogeneousMatrix res_h = start_h * rel_h;

  std::cout << "res_h: " << res_h << std::endl;
  std::cout << "res: " << vpPoseVector(res_h) << std::endl;

  vpHomogeneousMatrix res2_h = rel_h * start_h;

  std::cout << "res2_h: " << res2_h << std::endl;
  std::cout << "res2: " << vpPoseVector(res2_h) << std::endl;
  

  for (unsigned int i = 1; i <= ndata; i++) {
    std::ostringstream ss_fPe, ss_cPo;
    ss_fPe << "pose_fPe_" << i << ".yaml";
    ss_cPo << "pose_cPo_" << i << ".yaml";
    std::cout << "Use fPe=" << ss_fPe.str() << ", cPo=" << ss_cPo.str() << std::endl;

    vpPoseVector wPe;
    if (wPe.loadYAML(ss_fPe.str(), wPe) == false) {
      std::cout << "Unable to read data from: " << ss_fPe.str() << std::endl;
      return EXIT_FAILURE;
    }
    wMe[i - 1] = vpHomogeneousMatrix(wPe);

    vpPoseVector cPo;
    if (cPo.loadYAML(ss_cPo.str(), cPo)  == false) {
      std::cout << "Unable to read data from: " << ss_cPo.str() << std::endl;
      return EXIT_FAILURE;
    }
    cMo[i-1] = vpHomogeneousMatrix(cPo);
  }

 
  int ret = vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

  vpPoseVector pre;
  std::string pre_n = "pre.yaml";
  if (pre.loadYAML(pre_n, pre)  == false) {
      std::cout << "Unable to read data from: " << pre_n << std::endl;
  } else {
    vpHomogeneousMatrix preh(pre);
    for (unsigned int i = 0; i <= ndata - 1; i++) {
      std::cout << "res " << i << ":" << vpPoseVector(wMe[i] * preh * cMo[i]) << std::endl;
    }
  }

  
  // save eMc
  std::ofstream file_eMc("eMc.txt");
  eMc.save(file_eMc);

  vpPoseVector pose_vec(eMc);
  std::cout << "\nSave eMc.yaml" << std::endl;
  pose_vec.saveYAML("eMc.yaml", pose_vec);

  std::cout << "\nOutput: Hand-eye calibration result " << ret << " : eMc estimated " << std::endl;
  std::cout << eMc << std::endl;
  vpThetaUVector ePc(eMc);
  std::cout << "theta U (deg): " << vpMath::deg(ePc[0]) << " " << vpMath::deg(ePc[1]) << " " << vpMath::deg(ePc[2]) << std::endl;

  return EXIT_SUCCESS;
}

