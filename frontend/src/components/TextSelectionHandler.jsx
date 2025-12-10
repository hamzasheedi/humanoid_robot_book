import React, { useEffect, useState } from 'react';

const TextSelectionHandler = ({ onTextSelected }) => {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      if (text) {
        setSelectedText(text);
        // Pass the selected text to the parent component
        onTextSelected(text);
      } else {
        setSelectedText('');
        onTextSelected('');
      }
    };

    // Add event listeners
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        // Clear selection when escape key is pressed
        window.getSelection().empty();
        setSelectedText('');
        onTextSelected('');
      }
    });

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [onTextSelected]);

  return (
    <div style={{ display: 'none' }}>
      {/* This component doesn't render anything visible */}
      {/* It just handles text selection events */}
    </div>
  );
};

export default TextSelectionHandler;