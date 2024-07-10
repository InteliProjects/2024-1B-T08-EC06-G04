import React from 'react';
import PropTypes from 'prop-types';


// Botão principal utilizado, pede uma label: string, uma url: string, um onClick(opcional): function(opcional)e um className: string
const Button = ({ label, url, onClick, type = 'button', className = '' }) => {
  const handleClick = () => {
    if (onClick) {
      onClick();
    }
    window.location.href = url;
  };

  return (
    <button
      type={type}
      onClick={handleClick}
      className={`px-4 py-2 bg-green-400 hover:bg-green-700 text-white  rounded-lg shadow-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-400 focus:ring-opacity-75 ${className}`}
    >
      {label}
    </button>
  );
};

// Define os tipos de propriedades que o componente Button pode receber
Button.propTypes = {
  label: PropTypes.string.isRequired,
  url: PropTypes.string.isRequired,
  onClick: PropTypes.func,
  type: PropTypes.oneOf(['button', 'submit', 'reset']),
  className: PropTypes.string,
};

export default Button;
