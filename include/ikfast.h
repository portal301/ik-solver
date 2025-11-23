// IKFast 공통 헤더 파일
// OpenRAVE IKFast 생성 코드용

#ifndef IKFAST_H
#define IKFAST_H

#include <vector>
#include <cmath>

#define IKFAST_VERSION 0x1000004c

namespace ikfast {

typedef double IkReal;

template <typename T>
class IkSolutionBase {
public:
    virtual ~IkSolutionBase() {}
    virtual void GetSolution(T* solution, const T* freevalues) const = 0;
    virtual const std::vector<int>& GetFree() const = 0;
};

template <typename T>
class IkSolutionListBase {
public:
    virtual ~IkSolutionListBase() {}
    virtual size_t GetNumSolutions() const = 0;
    virtual const IkSolutionBase<T>& GetSolution(size_t index) const = 0;
    virtual size_t AddSolution(std::vector<T>& vinfos, const std::vector<int>& vfree) = 0;
    virtual void Clear() = 0;
};

// IKFast 생성 코드에서 사용하는 솔루션 클래스
template <typename T>
class IkSolution : public IkSolutionBase<T> {
public:
    std::vector<T> _vbasesol;
    std::vector<int> _vfree;

    void GetSolution(T* solution, const T* freevalues) const override {
        for (size_t i = 0; i < _vbasesol.size(); ++i) {
            if (_vfree.size() > 0) {
                // free parameter 처리 (현재 미사용)
                solution[i] = _vbasesol[i];
            } else {
                solution[i] = _vbasesol[i];
            }
        }
    }

    const std::vector<int>& GetFree() const override {
        return _vfree;
    }
};

template <typename T>
class IkSolutionList : public IkSolutionListBase<T> {
public:
    std::vector<IkSolution<T>> _listsolutions;

    size_t GetNumSolutions() const override {
        return _listsolutions.size();
    }

    const IkSolutionBase<T>& GetSolution(size_t index) const override {
        return _listsolutions[index];
    }

    size_t AddSolution(std::vector<T>& vinfos, const std::vector<int>& vfree) override {
        IkSolution<T> sol;
        sol._vbasesol = vinfos;
        sol._vfree = vfree;
        _listsolutions.push_back(sol);
        return _listsolutions.size() - 1;
    }

    void Clear() override {
        _listsolutions.clear();
    }
};

}  // namespace ikfast

#endif  // IKFAST_H
