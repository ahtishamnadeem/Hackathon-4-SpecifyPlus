import { useState, useEffect, useRef } from 'react';

const useOnScreen = (options = {}) => {
  const ref = useRef();
  const [isIntersecting, setIntersecting] = useState(false);

  useEffect(() => {
    const element = ref.current;
    if (!element) return;

    const observer = new IntersectionObserver(([entry]) => {
      setIntersecting(entry.isIntersecting);
    }, options);

    observer.observe(element);

    return () => {
      if (observer) {
        observer.unobserve(element);
      }
    };
  }, [options]);

  return [ref, isIntersecting];
};

export default useOnScreen;