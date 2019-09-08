#include "semantic_bgkoctree_node.h"
#include <cmath>

namespace la3dm {

    /// Default static values
    float Occupancy::sf2 = 1.0f;
    float Occupancy::ell = 1.0f;
    int Occupancy::nc = 3;
    float Occupancy::free_thresh = 0.3f;
    float Occupancy::occupied_thresh = 0.7f;
    float Occupancy::var_thresh = 1000.0f;
    float Occupancy::prior_A = 0.5f;
    float Occupancy::prior_B = 0.5f;


    /*Occupancy::Occupancy(float A, float B) : m_A(Occupancy::prior_A + A), m_B(Occupancy::prior_B + B) {
        classified = false;
        float var = get_var();
        if (var > Occupancy::var_thresh)
            state = State::UNKNOWN;
        else {
            float p = get_prob();
            state = p > Occupancy::occupied_thresh ? State::OCCUPIED : (p < Occupancy::free_thresh ? State::FREE
                                                                                                   : State::UNKNOWN);
        }
    }*/

    float Occupancy::get_prob() const {
        return m_A / (m_A + m_B);
    }

    std::vector<float> Occupancy::get_probs() const {
      float sum = 0;
      for (auto it = m_.begin(); it != m_.end(); ++it)
        sum += *it;
      std::vector<float> probs (m_.size());
      for (int i = 0; i < probs.size(); ++i)
        probs[i] = m_[i] / sum;
      return probs;
    }

    std::vector<float> Occupancy::get_vars() const {
      float sum = 0;
      for (auto it = m_.begin(); it != m_.end(); ++it)
        sum += *it;
      std::vector<float> vars(m_.size());
      for (int i = 0; i < vars.size(); ++i)
        vars[i] = ((m_[i] / sum)  - (m_[i] / sum)*(m_[i] / sum)) / (sum + 1);
      return vars;
    }

    void Occupancy::update(std::vector<float>& ybars) {
      classified = true;
      for (int i = 0; i < ybars.size(); ++i)
        m_[i] += ybars[i];

      std::vector<float> probs = get_probs();

      // update semantics
      semantics = std::distance(probs.begin(), std::max_element(probs.begin(), probs.end()));

      // update state
      std::vector<float> vars = get_vars();
      //if (vars[semantics] > Occupancy::var_thresh)
         //state = State::UNKNOWN;
      //else if (semantics == 0)
      if (semantics == 0)
        state = State::FREE;
      else {
          float p = 1 - probs[0];
          state = p > Occupancy::occupied_thresh ? State::OCCUPIED : (p < Occupancy::free_thresh ? State::FREE
			  									 : State::UNKNOWN);
      }
    }


    void Occupancy::update(float abar, float bbar) {
        classified = true;
        m_A += abar;
        //std::cout << "ybar: " << ybar << std::endl;
        //std::cout << "kbar: " << kbar << std::endl;
        m_B += bbar;

        float var = get_var();
        //if (var > Occupancy::var_thresh)
            //state = State::UNKNOWN;
        //else {
        float p = get_prob();
        state = p > Occupancy::occupied_thresh ? State::OCCUPIED : (p < Occupancy::free_thresh ? State::FREE
                                                                                                   : State::UNKNOWN);
        //}
    }

    /*std::ofstream &operator<<(std::ofstream &os, const Occupancy &oc) {
        os.write((char *) &oc.m_A, sizeof(oc.m_A));
        os.write((char *) &oc.m_B, sizeof(oc.m_B));
        return os;
    }

    std::ifstream &operator>>(std::ifstream &is, Occupancy &oc) {
        float m_A, m_B;
        is.read((char *) &m_A, sizeof(m_A));
        is.read((char *) &m_B, sizeof(m_B));
        oc = SemanticOcTreeNode(m_A, m_B);
        return is;
    }

    std::ostream &operator<<(std::ostream &os, const Occupancy &oc) {
        return os << '(' << oc.m_A << ' ' << oc.m_B << ' ' << oc.get_prob() << ')';
    }*/
}
